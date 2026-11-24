"""Write the evaluated formula results into the xlsx as cached values (formulas are kept)."""
import sys, re, zipfile, pickle, math
from xml.sax.saxutils import escape
import openpyxl
src, pkl, dst = sys.argv[1:4]
v = pickle.load(open(pkl, "rb"))
def flat(x):
    while isinstance(x, list): x = x[0]
    return x
zz = zipfile.ZipFile(src); wbx = zz.read("xl/workbook.xml").decode(); rel = zz.read("xl/_rels/workbook.xml.rels").decode()
rid = dict(re.findall(r'<sheet[^>]*name="([^"]+)"[^>]*r:id="([^"]+)"', wbx)); tgt = {}
for m in re.finditer(r'<Relationship [^>]*>', rel):
    i = re.search(r'Id="([^"]+)"', m.group(0)).group(1); g = re.search(r'Target="([^"]+)"', m.group(0)).group(1); tgt[i] = g.split("/")[-1]
from xml.sax.saxutils import unescape
byfile = {tgt[r]: unescape(n).upper() for n, r in rid.items()}
print(byfile)
cells = {}
for k, x in v.items():
    m = re.match(r"'\[[^\]]+\](.+)'!([A-Z]+\d+)$", k)
    if m: cells[(m.group(1).upper(), m.group(2))] = flat(x)
zin = zipfile.ZipFile(src); zout = zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED); stat = {"miss": 0, "done": 0}
for item in zin.infolist():
    data = zin.read(item.filename)
    m = re.match(r"xl/worksheets/sheet(\d+)\.xml", item.filename)
    if m:
        name = byfile[item.filename.split("/")[-1]]; xml = data.decode("utf8")
        def fix(mm):
            attrs, ref, f = mm.group(1), mm.group(2), mm.group(3)
            val = cells.get((name, ref))
            if val is None or (isinstance(val, float) and math.isnan(val)): stat["miss"] += 1; return mm.group(0)
            attrs = re.sub(r'\st="[^"]*"', "", attrs); stat["done"] += 1
            if isinstance(val, str): return '<c%s t="str"><f>%s</f><v>%s</v></c>' % (attrs, f, escape(val))
            if isinstance(val, bool): return '<c%s t="b"><f>%s</f><v>%d</v></c>' % (attrs, f, val)
            return '<c%s><f>%s</f><v>%r</v></c>' % (attrs, f, float(val))
        xml = re.sub(r'<c((?:\s[^>]*?)?\sr="([A-Z]+\d+)"[^>]*)><f>(.*?)</f><v\s*/?>(?:</v>)?</c>', fix, xml)
        data = xml.encode("utf8")
    zout.writestr(item, data)
zout.close(); print("cached", stat)
