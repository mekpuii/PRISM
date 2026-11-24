# test_advisor.py - run (orch-env, from orchestrator/):  python3 test/test_advisor.py
# advisor.py WITHOUT the internet and without a key: a fake "anthropic" module stands in for the SDK. Checks: off unless
# enabled; asks once per new problem (never for "solved", never faster than MIN_GAP_S); sends the request the way the
# API expects (model, JSON schema, fallbacks, cached system prompt); turns the answer into advice; an API error is
# reported, never raised into the optimizer.
import json
import os
import sys
import time
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sent = []
ANSWER = {"diagnosis": "The 12 V rail collapses while the gripper squeezes; pushes then have no power.",
          "confidence": "high", "would_change_my_mind": "a dead push while the arm stands still",
          "checks_by_hand": ["feeder servo 5 V supply", "D12 plug"],
          "experiment": {"what": "push only in TRANSPORT for 10 picks", "how_long": "10 picks",
                         "tells_apart": "rail sag vs dead servo", "safe": True},
          "say_to_operator": "The supply is too weak for gripper and feeder together."}


class _Err(Exception):
    def __init__(self, msg="", status_code=500):
        super().__init__(msg)
        self.message, self.status_code = msg, status_code


fake = types.ModuleType("anthropic")
for name in ("AuthenticationError", "RateLimitError", "APIStatusError", "APIConnectionError"):
    setattr(fake, name, type(name, (_Err,), {}))
MODE = {"fail": False}


class _Messages:
    def create(self, **kw):
        sent.append(kw)
        if MODE["fail"]:
            raise fake.APIStatusError("overloaded", 529)
        block = types.SimpleNamespace(type="text", text=json.dumps(ANSWER))
        return types.SimpleNamespace(stop_reason="end_turn", content=[block], model=kw["model"])


class Anthropic:
    def __init__(self, **kw):
        self.beta = types.SimpleNamespace(messages=_Messages())


fake.Anthropic = Anthropic
sys.modules["anthropic"] = fake

import advisor as AD                                               # noqa: E402

AD.LOG = os.path.join(os.environ.get("TMPDIR", "/tmp"), "prism_test_advice.jsonl")
STORY = {"problem": "feeder servo does not move", "tried": ["feed 7: servo did not move (0 mV)"],
         "saw": "no current", "so": "servo not moving", "next": "alarm"}


def wait_answer(a, n):
    end = time.time() + 5
    while time.time() < end and len(sent) < n or (a.last is None and time.time() < end):
        time.sleep(0.02)
    time.sleep(0.05)


def main():
    os.environ.pop("PRISM_ADVISOR", None)
    off = AD.Advisor()
    assert off.off and not off.consider(STORY, {}), off.off
    print(f"off:      without PRISM_ADVISOR=1 it never asks ({off.off})   OK")

    os.environ["PRISM_ADVISOR"] = "1"
    now = [1000.0]
    a = AD.Advisor(clock=lambda: now[0])
    assert a.off is None
    assert a.consider(STORY, {"rail_v": {"GRIP": 2.1}})
    wait_answer(a, 1)
    kw = sent[0]
    assert kw["model"] == "claude-opus-5-5" and kw["fallbacks"] == "default", kw.keys()
    assert kw["output_config"]["format"]["type"] == "json_schema" and kw["system"][0]["cache_control"]
    assert "GRIP" in kw["messages"][0]["content"]
    assert a.last["advice"]["confidence"] == "high" and a.last["advice"]["experiment"]["safe"] is True, a.last
    print(f"asks:     Claude Opus 5.5, JSON schema, fallbacks, cached system prompt -> advice: "
          f"'{a.last['advice']['say_to_operator']}'   OK")

    assert not a.consider(STORY, {}), "asked twice about the same problem"
    assert not a.consider({"problem": "solved: the feeder pushes again"}, {}), "asked about a solved problem"
    assert not a.consider({"problem": "another one"}, {}), "asked again within 10 minutes"
    now[0] += AD.MIN_GAP_S + 1
    MODE["fail"] = True
    a.last = None
    assert a.consider({"problem": "another one"}, {})
    wait_answer(a, 2)
    assert "error" in a.last and "529" in a.last["error"], a.last
    print(f"limits:   once per problem, never 'solved', 10 min apart; an API error is only reported "
          f"('{a.last['error']}')   OK")
    print("\nall advisor tests passed")


if __name__ == "__main__":
    main()
