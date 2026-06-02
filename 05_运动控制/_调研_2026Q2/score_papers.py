#!/usr/bin/env python3
"""足式论文归一化加权评分 v1.0"""
import re
from pathlib import Path

VENUE_SCORE = {
    "T-RO": 100, "TRO": 100, "IJRR": 100, "RSS": 100,
    "Science Robotics": 100, "SR": 100,
    "RA-L": 80, "RAL": 80, "ICRA": 80, "CoRL": 80,
    "NeurIPS": 80, "ICML": 80, "ICLR": 80,
    "IROS": 60, "Humanoids": 60, "CDC": 60, "L4DC": 60,
    "IEEE Access": 40, "Access": 40,
}
CURRENT_YEAR = 2026

AUTO_INCLUDE = {
    "GlobalSmooth": "Best Paper HM (T-RO 2023)",
    "CTR": "Best Paper (IJRR 2025)",
}

def parse_venue(venue_str):
    if not venue_str or venue_str == "—":
        return 40, None
    for ws_tag in ["WS", "Workshop"]:
        if ws_tag in venue_str:
            return 40, None
    for key, score in VENUE_SCORE.items():
        if key in venue_str:
            m = re.search(r"20(\d{2})", venue_str)
            year = int("20" + m.group(1)) if m else None
            return score, year
    return 40, None

def parse_year_from_arxiv(arxiv_str):
    m = re.search(r"(\d{2})\d{2}\.\d+", arxiv_str)
    if m:
        yy = int(m.group(1))
        return 2000 + yy if yy < 50 else 1900 + yy
    return None

def percentile_bucket(values, val):
    sorted_v = sorted(values)
    n = len(sorted_v)
    rank = sum(1 for v in sorted_v if v < val)
    pct = rank / n if n > 0 else 0
    if pct >= 0.8: return 100
    if pct >= 0.6: return 80
    if pct >= 0.4: return 60
    if pct >= 0.2: return 40
    return 20

def main():
    path = Path(__file__).parent / "tmp_legged_mpc.md"
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()

    headers = []
    rows = []
    in_table = False
    for i, line in enumerate(lines):
        s = line.strip()
        if "|" in s and not in_table:
            if i + 1 < len(lines) and re.match(r"^\s*\|[\s\-:|]+\|\s*$", lines[i + 1]):
                headers = [c.strip() for c in s.split("|")[1:-1]]
                in_table = True
        elif in_table and re.match(r"^\s*\|[\s\-:|]+\|\s*$", s):
            continue
        elif in_table and "|" in s:
            cells = [c.strip() for c in s.split("|")[1:-1]]
            row = {}
            for j, h in enumerate(headers):
                if j < len(cells):
                    row[h] = cells[j]
            rows.append(row)
        elif in_table and not s:
            continue

    papers = []
    for row in rows:
        short = row.get("简称", "?")
        venue_str = row.get("录用", "—")
        arxiv = row.get("arXiv", "—")
        gs_str = row.get("GS引用", "—")
        star_str = row.get("Star", "—")

        v_score, v_year = parse_venue(venue_str)
        if v_year is None:
            v_year = parse_year_from_arxiv(arxiv) or CURRENT_YEAR

        gs = 0
        has_citation = True
        if gs_str and gs_str != "—":
            try: gs = int(gs_str)
            except: gs = 0
        else:
            has_citation = False

        star = None
        if star_str and star_str != "—":
            try: star = int(star_str)
            except: star = None

        age = CURRENT_YEAR - v_year + 1
        annual_cite = gs / age if age > 0 else 0

        papers.append({
            "short": short,
            "venue": venue_str,
            "v_score": v_score,
            "year": v_year,
            "gs": gs,
            "has_citation": has_citation,
            "annual_cite": annual_cite,
            "star": star,
            "auto": AUTO_INCLUDE.get(short),
        })

    all_annual = [p["annual_cite"] for p in papers]
    star_papers = [p for p in papers if p["star"] is not None]
    all_stars = [p["star"] for p in star_papers]

    for p in papers:
        p["cite_pct"] = percentile_bucket(all_annual, p["annual_cite"])
        if p["star"] is not None:
            p["star_pct"] = percentile_bucket(all_stars, p["star"])
            p["final"] = 0.3 * p["v_score"] + 0.4 * p["cite_pct"] + 0.3 * p["star_pct"]
            p["dims"] = 3
        else:
            p["star_pct"] = None
            p["final"] = 0.5 * p["v_score"] + 0.5 * p["cite_pct"]
            p["dims"] = 2

    papers.sort(key=lambda p: (-1000 if p["auto"] else 0, -p["final"], -p["annual_cite"]))

    print("=" * 120)
    print(f"  足式MPC/优化控制 论文归一化加权评分 ({len(papers)} papers)")
    print(f"  三维: 0.3×Venue + 0.4×Citation + 0.3×Star | 二维: 0.5×Venue + 0.5×Citation")
    print("=" * 120)
    print()
    print(f"{'#':>3} {'简称':<22} {'录用':<18} {'V':>4} {'GS':>5} {'年均':>6} {'C%':>4} {'Star':>5} {'S%':>4} {'D':>2} {'得分':>6} {'备注'}")
    print("-" * 120)

    for i, p in enumerate(papers, 1):
        star_s = str(p["star"]) if p["star"] is not None else "—"
        star_pct_s = str(p["star_pct"]) if p["star_pct"] is not None else "—"
        note = ""
        if p["auto"]:
            note = f"★ {p['auto']}"
        elif not p["has_citation"]:
            note = "(无引用数据)"
        print(f"{i:3d} {p['short']:<22} {p['venue']:<18} {p['v_score']:4d} {p['gs']:5d} {p['annual_cite']:6.1f} {p['cite_pct']:4d} {star_s:>5} {star_pct_s:>4} {p['dims']:2d} {p['final']:6.1f} {note}")

    print()
    print("=" * 120)
    cite_sorted = sorted(all_annual)
    n = len(cite_sorted)
    print(f"  年均引用分位点: P20={cite_sorted[n//5]:.1f} P40={cite_sorted[2*n//5]:.1f} "
          f"P60={cite_sorted[3*n//5]:.1f} P80={cite_sorted[4*n//5]:.1f} max={cite_sorted[-1]:.1f}")
    if all_stars:
        star_sorted = sorted(all_stars)
        ns = len(star_sorted)
        print(f"  Star分位点: P20={star_sorted[ns//5]} P40={star_sorted[2*ns//5]} "
              f"P60={star_sorted[3*ns//5]} P80={star_sorted[4*ns//5]} max={star_sorted[-1]}")

    above_70 = sum(1 for p in papers if p["final"] >= 70 or p["auto"])
    above_50 = sum(1 for p in papers if p["final"] >= 50 or p["auto"])
    print(f"  >=70分: {above_70} 篇 | >=50分: {above_50} 篇 | 自动入选: {sum(1 for p in papers if p['auto'])} 篇")
    print("=" * 120)

if __name__ == "__main__":
    main()
