"""Totals bom.csv -> bom/BOM.md. Run: python bom/build_bom.py"""
import csv, os, collections
HERE = os.path.dirname(os.path.abspath(__file__))
LIMIT = 2000.0
def load():
    return list(csv.DictReader(open(os.path.join(HERE, "bom.csv"))))
def total(rows=None):
    return sum(int(r["qty"]) * float(r["unit_usd"]) for r in (rows or load()))
def main():
    rows = load(); groups = collections.OrderedDict()
    for r in rows: groups.setdefault(r["group"], []).append(r)
    L = ["# Bill of Materials", "", "Prices are retail estimates (Oct 2026), USD, shipping/tax/contingency are explicit lines. Re-verify before ordering.", ""]
    for g, rs in groups.items():
        sub = sum(int(r["qty"]) * float(r["unit_usd"]) for r in rs)
        L += [f"## {g}  (${sub:,.2f})", "", "| Ref | Item | Spec | Qty | Unit $ | Line $ | Notes |", "|---|---|---|--:|--:|--:|---|"]
        for r in rs:
            L.append(f"| {r['ref']} | {r['item']} | {r['spec']} | {r['qty']} | {float(r['unit_usd']):.2f} | {int(r['qty'])*float(r['unit_usd']):.2f} | {r['notes']} |")
        L.append("")
    t = total(rows)
    L += [f"## TOTAL: ${t:,.2f}  (budget ${LIMIT:,.0f}, margin ${LIMIT-t:,.2f})", ""]
    open(os.path.join(HERE, "BOM.md"), "w").write("\n".join(L))
    print(f"total ${t:,.2f}  margin ${LIMIT-t:,.2f}")
if __name__ == "__main__":
    main()
