# advisor.py - ask Claude about a problem the optimizer cannot explain (PRISM orchestrator, 2026-11-12).
#
# The operator: "make my optimizer smart like you, ready to make new things". The optimizer collects facts (its
# story: what it tried, what it saw, the rail per arm phase, the push evidence, its own experiments); when a NEW problem
# appears, this sends those facts to Claude and gets back, as JSON: a diagnosis, what to check by hand, and ONE
# suggested experiment. It ADVISES ONLY: nothing is applied by itself - the operator reads it (dashboard / log /
# logs/advice.jsonl) and decides. It never blocks the control loop (a background thread), asks at most once per
# MIN_GAP_S and MAX_PER_DAY times a day, and is OFF unless enabled.
#
# ENABLE (two steps, once):
#   ~/orch-env/bin/pip install anthropic
#   a key: export ANTHROPIC_API_KEY=... (or `ant auth login`), then start the optimizer with PRISM_ADVISOR=1
# Cost: one question is a few thousand tokens on Claude Opus 5.5 ($4 / $20 per million tokens) - about 2-5 US cents.

import json
import os
import queue
import threading
import time

MODEL = "claude-opus-5-5"
MIN_GAP_S = 600.0           # at most one question per 10 minutes
MAX_PER_DAY = 20
TIMEOUT_S = 120.0
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "advice.jsonl")

SYSTEM = """You advise the optimizer of PRISM, a small robot cell on a Raspberry Pi: a feeder pushes cubes (40 mm,
white or orange) from a stack onto a belt; the belt brings them to a pick IR; a 6-joint arm (STS3215 servos) grips each
cube and drops it into a bin by colour. A camera tracks cubes on the belt. The optimizer runs "flow": it decides every
push of the feeder, verifies each push (feeder servo current as a 12 V rail dip, the camera seeing a new cube at the
feeder exit, the feeder IR), keeps 1-3 cubes in flight, learns which arm phases are good moments to push and how long a
push must be, and experiments with its own knobs. The arm and the feeder share one power supply.

You get the optimizer's own account of a problem it could not explain: what it tried, what it saw, its conclusion and
its next step, plus measurements. Your job:
- Say what most likely happened, using only the facts given; say how sure you are, and what fact would change your mind.
- List concrete checks a person can do by hand at the cell (plugs, supplies, mechanics, light), most useful first.
- Suggest ONE experiment the optimizer could run that would tell the possible causes apart. It must be safe: no new
  arm poses, no speeds outside what it already uses, nothing that bypasses its safety rules (belt held while the arm
  grips, no cube at the IR before the arm is parked, the alarm on a dead servo). If no safe experiment helps, say so.
- Write for the operator: short, plain sentences (English is not their first language).
Never claim something was measured that is not in the facts."""

SCHEMA = {
    "type": "object",
    "properties": {
        "diagnosis": {"type": "string", "description": "what most likely happened, 1-3 sentences"},
        "confidence": {"type": "string", "enum": ["low", "medium", "high"]},
        "would_change_my_mind": {"type": "string"},
        "checks_by_hand": {"type": "array", "items": {"type": "string"}},
        "experiment": {
            "type": "object",
            "properties": {
                "what": {"type": "string", "description": "the change, concretely"},
                "how_long": {"type": "string"},
                "tells_apart": {"type": "string", "description": "which causes it separates and how"},
                "safe": {"type": "boolean"},
            },
            "required": ["what", "how_long", "tells_apart", "safe"],
            "additionalProperties": False,
        },
        "say_to_operator": {"type": "string", "description": "one or two plain sentences for the dashboard"},
    },
    "required": ["diagnosis", "confidence", "would_change_my_mind", "checks_by_hand", "experiment", "say_to_operator"],
    "additionalProperties": False,
}


def enabled_reason():
    """None if the advisor can run, else why not (shown in the plan score)."""
    if os.environ.get("PRISM_ADVISOR") != "1":
        return "off (start the optimizer with PRISM_ADVISOR=1)"
    try:
        import anthropic  # noqa: F401
    except ImportError:
        return "the anthropic package is not installed (~/orch-env/bin/pip install anthropic)"
    return None


class Advisor:
    def __init__(self, clock=time.time):
        self.clock = clock
        self.off = enabled_reason()
        self.last = None                  # the newest answer: {"problem", "t", "advice" | "error"}
        self._q = queue.Queue(maxsize=1)
        self._asked = []                  # times of questions in the last day
        self._seen = set()                # problems already asked about (once each)
        if self.off is None:
            threading.Thread(target=self._loop, daemon=True, name="advisor").start()

    # ----- from the optimizer -----
    def consider(self, story, facts):
        """A new story (not a "solved" one). Queues ONE question if allowed; never blocks."""
        if self.off is not None or not story or str(story.get("problem", "")).startswith("solved"):
            return False
        key = story.get("problem")
        now = self.clock()
        self._asked = [t for t in self._asked if now - t < 86400]
        if key in self._seen or len(self._asked) >= MAX_PER_DAY or (self._asked and now - self._asked[-1] < MIN_GAP_S):
            return False
        try:
            self._q.put_nowait((story, facts))
        except queue.Full:
            return False
        self._seen.add(key)
        self._asked.append(now)
        return True

    def summary(self):
        return {"off": self.off} if self.off else (self.last or {"waiting": True})

    # ----- the question (background thread) -----
    def _loop(self):
        import anthropic
        client = anthropic.Anthropic(timeout=TIMEOUT_S, max_retries=2)
        while True:
            story, facts = self._q.get()
            out = {"problem": story.get("problem"), "t": self.clock()}
            try:
                out["advice"] = self._ask(client, story, facts)
            except anthropic.AuthenticationError:
                out["error"] = "no valid API key (ANTHROPIC_API_KEY or `ant auth login`)"
            except anthropic.RateLimitError:
                out["error"] = "rate limited by the API - asked again on the next new problem"
                self._seen.discard(story.get("problem"))
            except anthropic.APIStatusError as e:
                out["error"] = f"API error {e.status_code}: {e.message}"
            except anthropic.APIConnectionError:
                out["error"] = "no connection to the API (internet?)"
            except (ValueError, KeyError) as e:
                out["error"] = f"unusable answer: {e}"
            self.last = out
            try:
                os.makedirs(os.path.dirname(LOG), exist_ok=True)
                with open(LOG, "a") as f:
                    f.write(json.dumps({**out, "story": story, "facts": facts}, default=str) + "\n")
            except OSError:
                pass

    def _ask(self, client, story, facts):
        text = ("THE OPTIMIZER'S ACCOUNT\n" + json.dumps(story, indent=1, default=str)
                + "\n\nMEASUREMENTS AND STATE\n" + json.dumps(facts, indent=1, default=str))
        response = client.beta.messages.create(
            model=MODEL,
            max_tokens=16000,
            betas=["server-side-fallback-2026-07-01"],
            fallbacks="default",                              # a declined request is re-run on a fallback model
            output_config={"effort": "high", "format": {"type": "json_schema", "schema": SCHEMA}},
            system=[{"type": "text", "text": SYSTEM, "cache_control": {"type": "ephemeral"}}],
            messages=[{"role": "user", "content": text}],
        )
        if response.stop_reason == "refusal":
            raise ValueError("the model declined")
        if response.stop_reason == "max_tokens":
            raise ValueError("answer cut off")
        body = next(b.text for b in response.content if b.type == "text")
        advice = json.loads(body)
        advice["model"] = response.model
        return advice
