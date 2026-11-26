"""Characterization suite for suspension/xlsx_formula_cache.py (mutation gate); see tests/_golden.py. One workbook
with several hundred formulas across two sheets (every supported function on numbers, text, booleans, blanks,
errors and ranges; every operator and comparison; precedence, unary minus, percent and concatenation; cross-sheet,
absolute and whole-range references; MATCH in all three modes with wildcards; TEXT formats; rounding modes;
unsupported functions, bad references and circular chains), evaluated and written back as cached values."""
import shutil
import zipfile

import openpyxl
import pytest

from suspension import xlsx_formula_cache as X
from tests._golden import plain
from tests._golden_lazy import check_lazy

DATA = {"A1": 1.0, "A2": 2.5, "A3": -4.0, "A4": 0.0, "A5": "text", "A6": True, "A7": None, "A8": "12", "A9": "b*c",
        "B1": 10, "B2": 20, "B3": 30, "B4": 40, "B5": 50, "C1": "apple", "C2": "banana", "C3": "cherry",
        "C4": "date", "C5": "elder", "D1": 1, "D2": 3, "D3": 5, "D4": 7, "D5": 9, "E1": False, "E2": "#N/A",
        "A10": "aX", "A11": "a~X", "A12": "a~", "A13": "b*z", "A14": "b*", "M1": True, "M2": 5, "M3": "x", "M4": 7,
        "I1": 50, "I2": 40, "I3": 30, "I4": 20, "I5": 10, "Z1": 1, "AA1": 2, "AB1": 3,
        "G6": "=1/0", "G7": '=""', "G8": "=$G$8+1", "J2": "=NOSUCH(1)", "K2": "=J2+1", "J3": "=NOSUCH(2)", "K3": "=1+1",
        "O1": "=B1:B2", "P1": "=1+2", "O2": "=SQRT()", "P2": "=2+2"}

FORMULAS = [
    "=1+2*3", "=(1+2)*3", "=2^3^2", "=-2^2", "=--3", "=+4", "=10/4", "=1/0", "=50%", "=5%*20", "=\"a\"&\"b\"&1",
    "=1&TRUE", "=A1+A2", "=A1-A3*A2", "=A5+1", "=A6+1", "=A7+1", "=A8+1", "=A8*2", "=E1*3", "=$A$1+A$2+$A3",
    "=Data!B1+Data!$B$2", "='Data'!B3", "=1=1", "=1<>1", "=2<3", "=3<=3", "=4>5", "=5>=6", "=\"a\"<\"b\"",
    "=\"B\"=\"b\"", "=\"10\">9", "=TRUE>1", "=A7=0", "=A7=\"\"", "=1<\"a\"", "=FALSE<TRUE",
    "=SUM(B1:B5)", "=SUM(B1:B5,A1,2)", "=SUM(A1:A9)", "=SUM(\"3\",1)", "=SUM()", "=SUM(Data!B1:B3)",
    "=AVERAGE(B1:B5)", "=AVERAGE(A5)", "=AVERAGE(D1:D5,1)", "=MAX(B1:B5,A3)", "=MAX(A5)", "=MIN(B1:B5,A3)",
    "=MIN()", "=COUNT(A1:A9)", "=COUNT(1,\"2\",TRUE)", "=COUNTA(A1:A9)", "=COUNTA(A7)", "=SUMPRODUCT(B1:B5,D1:D5)",
    "=SUMPRODUCT(B1:B3,D1:D2)", "=SUMPRODUCT(B1:B5)", "=SUMPRODUCT(B1:B2*D1:D2)",
    "=INDEX(B1:B5,3)", "=INDEX(B1:D5,2,3)", "=INDEX(B1:B5,0)", "=INDEX(B1:B5,9)", "=INDEX(B1:D5,2)",
    "=INDEX(B1:D5,0,2)", "=INDEX(C1:C5,1,1)", "=INDEX(B1:B5,-1)",
    "=MATCH(30,B1:B5,0)", "=MATCH(35,B1:B5,1)", "=MATCH(35,B1:B5)", "=MATCH(5,B1:B5,1)", "=MATCH(99,B1:B5,0)",
    "=MATCH(\"cher*\",C1:C5,0)", "=MATCH(\"?anana\",C1:C5,0)", "=MATCH(\"b~*c\",A9,0)", "=MATCH(\"BANANA\",C1:C5,0)",
    "=MATCH(25,{50,40,30,20,10},-1)", "=MATCH(60,B1:B5,-1)", "=MATCH(1,B1:C2,0)", "=MATCH(\"x\",C1:C5,1)",
    "=TEXT(1234.567,\"0.00\")", "=TEXT(1234.567,\"#,##0\")", "=TEXT(0.256,\"0%\")", "=TEXT(0.256,\"0.0%\")",
    "=TEXT(5,\"0\")", "=TEXT(-3.14159,\"0.000\")", "=TEXT(2.5,\"#.#\")", "=TEXT(1,\"yyyy\")", "=TEXT(\"a\",\"0\")",
    "=ROUND(2.5,0)", "=ROUND(-2.5,0)", "=ROUND(1.2345,2)", "=ROUND(1234,-2)", "=ROUNDUP(1.21,1)", "=ROUNDUP(-1.21,1)",
    "=ROUNDDOWN(1.29,1)", "=ROUNDDOWN(-1.29,1)", "=ROUND(A5,1)", "=ROUND(1.5)",
    "=SQRT(16)", "=SQRT(-1)", "=SQRT(A5)", "=POWER(2,10)", "=POWER(-8,1/3)", "=POWER(0,-1)", "=POWER(4,0.5)",
    "=ABS(A3)", "=INT(-2.5)", "=INT(2.9)", "=SIGN(A3)", "=SIGN(0)", "=SIGN(7)", "=PI()",
    "=AND(TRUE,1)", "=AND(A1:A2,FALSE)", "=OR(FALSE,0)", "=OR(E1,A1)", "=NOT(0)", "=NOT(A5)", "=AND()",
    "=IF(1,2,3)", "=VLOOKUP(1,B1:B5,1)", "=NOSUCH(1)", "=SUM(ZZZ9999999)", "=Missing!A1", "=A1+", "=(1+2",
    "=#REF!", "=#DIV/0!+1", "=E2", "=1E3+1", "=.5*2", "=SUM(B:B)", "=Formulas!A1", "=G1", "=H1", "=G3",
    "=\"he said \"\"hi\"\"\"", "=MAX(-1E300*1E10)", "=1E308*10", "=SUM(1,#N/A)", "=MIN(B1:B5)-MAX(D1:D5)",
]

FORMULAS2 = [
    "= 1 + 2 ", "=1 +2", "=SUM( 1 , 2 )", "=-(1/0)", "=-#N/A", '=""+1', "=B1:B2+1", "=B1:B1+1", "=B1:C1+1", "=-A1",
    '="a"&G6', '="a"&A7', '="a"&TRUE', '="a"&FALSE', '="a"&1E15', '="a"&1E16', '="a"&2.5', '="a"&(0-0)', '="a"&B1:B2',
    '="a"&999999999999999', "=1&G6", "=G6&1", "=1+#N/A", "=#N/A+1", "=#NULL!",
    "=IF(G6,1,2)", "=IF(A7,1,2)", '=IF("true",1,2)', '=IF(" false ",1,2)', '=IF("x",1,2)', "=IF(B1:B2,1,2)",
    "=IF(0.5,1,2)", "=IF(A1,B1,B2)", "=IF(A4,B1,B2)", "=IF(A4,B1)", "=IF(1)", "=IF(1,2,3,4)", "=IF(0,1)",
    "=IFERROR(1)", "=IFERROR(1/0,B1)", "=IFERROR(#N/A,B2)", "=IFERROR(SQRT(-1),B3)", "=IFERROR(B4,5)", "=IFERROR(G6,B5)",
    "=IFNA(#N/A,B1)", "=IFNA(MATCH(99,B1:B5,0),B2)", "=IFNA(1/0,3)", "=IFNA(B3,3)", "=IFNA(1)", "=IFNA(E2,4)",
    "=ISERROR(G6)", "=ISERROR(1/0)", "=ISERROR(B1)", "=ISNA(#N/A)", "=ISNA(MATCH(99,B1:B5,0))", "=ISNA(G6)",
    "=ISBLANK(A7)", "=ISBLANK(A1)", "=ISBLANK(G6)", "=ISNUMBER(A1)", "=ISNUMBER(A6)", "=ISNUMBER(A5)", "=ISTEXT(A5)",
    "=ISTEXT(A1)", "=ISTEXT()", "=ISBLANK(B1:B2)", "=ISBLANK(B1:B1)",
    "=SUM(G5:G6)", "=SUM(M1:M4)", "=SUM(A7,5)", '=COUNT("",1)', '=SUM("",1)', "=SUM(TRUE,1)", "=SUM(FALSE,1)",
    "=SUM(1/0)", "=COUNT(A7,1)", "=AVERAGE(A7)", "=AVERAGE(C1:C5)", "=MAX(C1:C5)", "=MIN(C1:C5)", "=MAX(M1:M4)",
    "=COUNTA(A1:A3,B1:B2)", "=COUNTA(G7)", "=COUNTA(G6:G7)", "=COUNTA(A7:A8,G7)",
    "=SUMPRODUCT()", "=SUMPRODUCT(2,3)", "=SUMPRODUCT(G5:G6)", "=SUMPRODUCT(M1:M4)", "=SUMPRODUCT(C1:C2)",
    "=SUMPRODUCT(M1:M4,I1:I4)",
    "=INDEX()", "=INDEX(B1:B5)", "=INDEX(5,1)", "=INDEX(5,1,1)", "=INDEX(B1:D1,2)", "=INDEX(B1:D1,4)",
    "=SUM(INDEX(B1:D5,0,0))", "=SUM(INDEX(B1:D5,0,2))", "=INDEX(B1:D5,0,4)", "=INDEX(B1:D5,0,0)",
    "=SUM(INDEX(B1:D5,2,0))", "=SUM(INDEX(B1:D5,1,0))", "=SUM(INDEX(B1:D5,5,0))", "=INDEX(B1:D5,6,0)",
    "=INDEX(B1:D5,5,1)", "=INDEX(B1:D5,5,3)", "=INDEX(B1:D5,1,4)", "=INDEX(B1:D5,6,1)", "=SUM(INDEX(B1:D5,0,3))",
    "=INDEX(B1:D5,-1,1)", "=SUM(INDEX(B1:D5,0))", "=SUM(INDEX(B1:D5,0,1))",
    "=MATCH(1)", '=MATCH("zzz",C1:C5,0)', "=MATCH(6,M1:M4,1)", "=MATCH(30,B1:B5,1)", "=MATCH(25,I1:I5,-1)",
    "=MATCH(5,I1:I5,-1)", "=MATCH(60,I1:I5,-1)", "=MATCH(30,I1:I5,-1)", '=MATCH("*~X",A10:A11,0)',
    '=MATCH("*a~",A12,0)', '=MATCH("b~*",A13:A14,0)', '=MATCH("CHER*",C1:C5,0)', '=MATCH("?ANANA",C1:C5,0)',
    "=TEXT(1)", '=TEXT(1234.5,"0.0;(0.0)")', '=TEXT(-1234.5,"0.0;(0.0)")', '=TEXT(0.5,"#.00")', '=TEXT(-0.5,"#.0")',
    '=TEXT(10.5,"#.0")', '=TEXT(0.5,"0.0")', '=TEXT(0.5,"#,##0.00")', '=TEXT(0.25,"#.0%")', '=TEXT(TRUE,"0")',
    '=TEXT(A7,"0")', '=TEXT(1,"@")', '=TEXT(G6,"0")', "=TEXT(1,G6)", '=TEXT(B1:B2,"0")', '=TEXT(1234567,"#,##0")',
    "=ROUNDUP(0.05,1)", "=ROUNDUP(0,1)", "=ROUNDDOWN(0.05,1)", "=ROUND()", "=ROUND(1,2,3)", "=ROUNDUP(-0.05,1)",
    "=SQRT(0)", "=SQRT(0.25)", "=SQRT()", "=POWER(2)", "=POWER(0,0)", "=POWER(-8,2)",
    "=A7=A7", "=A7<1", "=A7>-1", "=1>A7", '="a">A7', '=A7<"a"', "=TRUE=1", "=1=TRUE", '="a">1', '=1<"a"', '="a"="A"',
    "=A7<>A7", "=A7>=A7", "=A7<=0", "=TRUE>FALSE", '="b">="a"', '=TRUE<"a"',
    "=AA1", "=SUM(Z1:AB1)", "=SUM(B5:B1)", "=SUM(D1:B1)", "=SUM(D5:B1)", "=SUM(B1:D2)", "='We!rd'!A1+1",
    "='It''s'!A1+1", "=Other!A1+A1",
    "=INDEX(5,0,0)+1", "=INDEX(B1:B1,0,0)*2", "=SUM(AA1:AB1)", "=$B$1:B1+1", "=G7=A7", "=A7=G7", "=A7<A7", "=A7>A7",
    '=1<"!"', '="!">1', "=2=1", "=1<1", "=2>2", '="a"<>""', "=SUMPRODUCT(M1:M4,I1:I4)", "=COUNT(M1:M4)",
    "=A7<=A7", "=2>=2", "=1>=2",
]


@pytest.fixture(scope="module")
def book(tmp_path_factory):
    d = tmp_path_factory.mktemp("xl")
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "Data"
    for k, v in DATA.items():
        ws[k] = v
    for i, f in enumerate(FORMULAS + FORMULAS2, start=1):
        ws[f"F{i}"] = f
    ws["L1"] = "=not a formula"
    ws["L1"].data_type = "s"
    from openpyxl.worksheet.formula import ArrayFormula
    ws["N1"] = ArrayFormula("N1:N2", "=B1:B2*2")
    ws["N3"] = "=N1+1"
    ws["G1"] = "=H1+1"            # a two-cell cycle
    ws["H1"] = "=G1+1"
    ws["G2"] = "=G2"              # a self reference
    ws["G3"] = "=SUM(Data!G4:G5)"
    ws["G4"] = "=2*3"
    ws["G5"] = 4
    fs = wb.create_sheet("Formulas")
    fs["A1"] = "=Data!F1*2"
    fs["A2"] = "=Data!F8"
    other = wb.create_sheet("KX Sheet")
    other["A1"] = "=Data!B5/Data!B1"
    other["A2"] = "=A1*2"
    for title, v in (("We!rd", 11), ("It's", 12)):
        wb.create_sheet(title)["A1"] = v
    o = wb.create_sheet("Other")
    for k, v in {"A1": 100, "B1": 200, "A2": "=IF(A1,B1,0)", "A3": "=IFERROR(1/0,B1)", "A4": "=IFNA(#N/A,B1)",
                 "A5": "=-A1", "A6": "=ISNUMBER(A1)", "A7": "=SUM(A1:B1)", "A8": "=IF(0,1,B1)", "A9": "=IFNA(B1,0)",
                 "A10": "=IFERROR(B1,0)"}.items():
        o[k] = v
    p = d / "book.xlsx"
    wb.save(p)
    return p


def _values(vals):
    return {f"{s}!{c}": plain(v.text if isinstance(v, X.XlError) else v) for (s, c), v in sorted(vals.items())}


def _cached(path):
    wb = openpyxl.load_workbook(path, data_only=True)
    return {f"{ws.title}!{c.coordinate}": plain(c.value) for ws in wb.worksheets for row in ws.iter_rows()
            for c in row if c.value is not None}


def _raises(fn, *a):
    try:
        return ["ok", _values({("r", "r"): fn(*a)})["r!r"]]
    except Exception as e:                                          # noqa: BLE001
        return [type(e).__name__, str(e), getattr(getattr(e, "err", None), "text", None)]


def _cases(book, tmp_path):
    X._parse.cache_clear()      # mutmut forks each mutant from a warmed process: parse under the mutant
    vals, rep = X.evaluate_workbook(str(book))
    yield "values", _values(vals)
    yield "report", plain(rep)
    vals2, rep2 = X.evaluate_workbook(str(book), only_prefix="KX ")
    yield "kx", [_values(vals2), plain(rep2)]
    vals3, rep3 = X.evaluate_workbook(str(book), max_cells=5)
    yield "capped", [_values(vals3), plain(rep3)]
    for name, kw in (("all", {}), ("kx", dict(only_prefix="KX ")), ("none", dict(only_prefix="Nope"))):
        p = tmp_path / f"{name}.xlsx"
        shutil.copy(book, p)
        yield f"populate.{name}", [plain(X.populate_cached_values(str(p), **kw)), _cached(p),
                                   sorted(zipfile.ZipFile(p).namelist())]
    yield "errors", [[e.text, repr(e), e == X.XlError(e.text), hash(e) == hash(X.XlError(e.text))]
                     for e in (X.DIV0, X.VALUE, X.REF, X.NAME, X.NUM, X.NA)]
    yield "supported", sorted(X.SUPPORTED_FUNCTIONS)
    yield "errors.eq", [[e == 5, e == e.text, e == X.XlError(e.text), e != X.NA] for e in (X.DIV0, X.NA)]
    yield "parse", [_raises(X._parse, f) for f in ("1 @ 2", "1+", "(1+2", "(1+2,3)", "SUM(1 2)", ")", "1 2", "SUM(1", "",
                                                    "#NULL!", "1+" + "@" * 30, "SUM(,)", "1 +", "-", "SUM(1,")]
    yield "text.direct", [_raises(X._f_text, a) for a in ([1.0, "@"], [1.0, "0.0;x"], [1.0, "0.0.0"], [1.0])]
    wb = openpyxl.load_workbook(book)
    ev = X._Evaluator(wb, max_cells=4)
    yield "evaluator", [_raises(ev.range_value, "Data", "B1", "D2"), _raises(ev.range_value, "Data", "B1", "C2"),
                        _raises(ev.evaluate, "1+2", "Data"), _raises(ev.evaluate, "=1+2", "Data"),
                        _raises(ev.evaluate, "=A1", "Data"), _raises(ev.evaluate, "=A1", "Other"),
                        _raises(ev.sheet, "Nope", "Data"), _raises(lambda: ev.sheet(None, "Data").title),
                        _raises(ev.evaluate, "=NOSUCH(1)", "Data"), _raises(X._split_ref, "A1:ZZZZ9"),
                        _raises(X._split_ref, "'It''s'!B2:C3"), _raises(X._split_ref, "Data!A1")]
    wb.close()
    yield "cached_repr", [X._cached_repr(v) for v in (X.NA, True, False, 1, 2.5, float("nan"), float("inf"), 1e15,
                                                        1e15 - 1, -3.0, -1e15, "s", None, object(), [1])]
    yield "escape", X._xml_escape("a&b<c>d\x01e\tf\ng\rh\x7f").decode()
    xml = (b'<sheetData><row r="1"><c r="A1"/><c><f>1</f></c><c s="1"><f>1</f></c><c r="A2"><f>1+1</f></c>'
           b'<c r="A3"><v>5</v></c><c r="A4"><f t="shared" ref="A4:A5" si="0">1</f></c><c r="A5"><f t="shared" si="0"/>'
           b'</c><c r="A6"><f t="array" ref="A6">1</f></c><c r="A7"><f t="dataTable" ref="A7"/></c>'
           b'<c r="A8" s="2" t="str"><f>1</f><v>old</v></c><c r="A9"><f>x</f></c><c r="A10"><f>x</f></c>'
           b'<c r="A11" t="n"><f>x</f></c><c r="A12"><f>x</f></c><c r="A13"><f>x</f></c><c r="A14"><f a="1">x</f></c>'
           b'<c r="A15"><f/></c></row></sheetData>')
    cells = {"A1": 1, "A3": 1, "A4": 1, "A5": 1, "A6": 1, "A7": 1, "A8": 2.5, "A9": None, "A10": object(), "A11": X.NA,
             "A12": "a&b<c>", "A13": True, "A14": 3, "A15": 4}
    out, n = X._inject(xml, cells)
    yield "inject", [out.decode(), n]
    zp = tmp_path / "paths.zip"
    with zipfile.ZipFile(zp, "w") as z:
        z.writestr("xl/workbook.xml", f'<workbook xmlns="{X._NS}" xmlns:r="{X._NS_REL}"><bookViews r:id="rId2" '
                   'name="ghost"/><sheets><sheet name="A" sheetId="1" r:id="rId1"/><sheet name="B" sheetId="2" '
                   'r:id="rId2"/><sheet name="C" sheetId="3" r:id="rId3"/><sheet name="D" sheetId="4" r:id="rId4"/>'
                   '<sheet name="E" sheetId="5" r:id="rId9"/><sheet name="F" sheetId="6" r:id="rId5"/>'
                   '<sheet name="G" sheetId="7" r:id="rId6"/></sheets></workbook>')
        z.writestr("xl/_rels/workbook.xml.rels", '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/'
                   'relationships"><Relationship Id="rId1" Target="worksheets/sheet1.xml"/><Relationship Id="rId2" '
                   'Target="/xl/worksheets/sheet2.xml"/><Relationship Id="rId3"/><Relationship Id="rId4" '
                   'Target=".//ws/s4./"/><Relationship Id="rId5" Target="Xl.xml"/><Relationship Id="rId6" '
                   'Target="/Xl/x.xml"/></Relationships>')
    with zipfile.ZipFile(zp) as z:
        yield "sheet_paths", X._sheet_xml_paths(z)
    cc = tmp_path / "calc.xlsx"
    shutil.copy(book, cc)
    tmpz = tmp_path / "calc2.xlsx"
    with zipfile.ZipFile(cc) as src, zipfile.ZipFile(tmpz, "w", zipfile.ZIP_DEFLATED) as dst:
        dst.writestr("xl/calcChain.xml", "<calcChain/>")
        for item in src.infolist():
            dst.writestr(item, src.read(item.filename))
    yield "populate.calcchain", [plain(X.populate_cached_values(str(tmpz))), sorted(zipfile.ZipFile(tmpz).namelist()),
                                 _cached(tmpz)]
    cap = tmp_path / "cap.xlsx"
    shutil.copy(book, cap)
    yield "populate.capped", [plain(X.populate_cached_values(str(cap), max_cells=5)), _cached(cap)]



def test_every_formula_evaluated_and_cached(book, tmp_path):
    check_lazy("xlsx_formula_cache", _cases(book, tmp_path))
