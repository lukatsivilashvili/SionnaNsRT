#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import pathlib
import shutil
import tempfile
import zipfile
from datetime import datetime, timezone
from xml.etree import ElementTree as ET
from xml.sax.saxutils import escape


ROOT = pathlib.Path("/home/luksona/WorkingFiles/sionna_files/SionnaNsRT")
TEMPLATE = pathlib.Path("/home/luksona/Downloads/template-cedric.potx")
OUT = ROOT / "project_current_state_sionna_ns3_sumo.pptx"
RUN = ROOT / "results/sionna_ns3_experiment_20260507_012424"
ASSET = ROOT / "presentation_assets"

EMU_W = 12192000
EMU_H = 6858000

NS_CT = "http://schemas.openxmlformats.org/package/2006/content-types"
NS_REL = "http://schemas.openxmlformats.org/package/2006/relationships"


def emu_x(percent: float) -> int:
    return int(EMU_W * percent / 100.0)


def emu_y(percent: float) -> int:
    return int(EMU_H * percent / 100.0)


def textbox(shape_id: int, x: float, y: float, w: float, h: float, text: str,
            size: int = 18, bold: bool = False, color: str = "222222") -> str:
    paragraphs = []
    for line in text.splitlines() or [""]:
        paragraphs.append(
            f'<a:p><a:r><a:rPr lang="en-US" sz="{size * 100}"'
            f'{" b=\"1\"" if bold else ""}>'
            f'<a:solidFill><a:srgbClr val="{color}"/></a:solidFill>'
            f'</a:rPr><a:t>{escape(line)}</a:t></a:r></a:p>'
        )
    return f"""
      <p:sp>
        <p:nvSpPr><p:cNvPr id="{shape_id}" name="TextBox {shape_id}"/><p:cNvSpPr txBox="1"/><p:nvPr/></p:nvSpPr>
        <p:spPr><a:xfrm><a:off x="{emu_x(x)}" y="{emu_y(y)}"/><a:ext cx="{emu_x(w)}" cy="{emu_y(h)}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:noFill/><a:ln><a:noFill/></a:ln></p:spPr>
        <p:txBody><a:bodyPr wrap="square" lIns="0" tIns="0" rIns="0" bIns="0"/><a:lstStyle/>{''.join(paragraphs)}</p:txBody>
      </p:sp>"""


def picture(shape_id: int, rel_id: str, x: float, y: float, w: float, h: float,
            name: str) -> str:
    return f"""
      <p:pic>
        <p:nvPicPr><p:cNvPr id="{shape_id}" name="{escape(name)}"/><p:cNvPicPr><a:picLocks noChangeAspect="1"/></p:cNvPicPr><p:nvPr/></p:nvPicPr>
        <p:blipFill><a:blip r:embed="{rel_id}"/><a:stretch><a:fillRect/></a:stretch></p:blipFill>
        <p:spPr><a:xfrm><a:off x="{emu_x(x)}" y="{emu_y(y)}"/><a:ext cx="{emu_x(w)}" cy="{emu_y(h)}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr>
      </p:pic>"""


def accent_bar() -> str:
    return f"""
      <p:sp>
        <p:nvSpPr><p:cNvPr id="900" name="CEDRIC Accent"/><p:cNvSpPr/><p:nvPr/></p:nvSpPr>
        <p:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{EMU_W}" cy="{emu_y(1.8)}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:solidFill><a:srgbClr val="C7012B"/></a:solidFill><a:ln><a:noFill/></a:ln></p:spPr>
      </p:sp>"""


def footer(slide_no: int) -> str:
    return (
        textbox(901, 4, 94, 72, 3, "ns-3 + Sionna RT + SUMO/TAP current state", 8, False, "666666")
        + textbox(902, 93, 94, 3, 3, str(slide_no), 8, False, "666666")
    )


def slide_xml(shapes: str) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sld xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
       xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
       xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:cSld>
    <p:bg><p:bgPr><a:solidFill><a:srgbClr val="FFFFFF"/></a:solidFill><a:effectLst/></p:bgPr></p:bg>
    <p:spTree>
      <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
      <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{EMU_W}" cy="{EMU_H}"/><a:chOff x="0" y="0"/><a:chExt cx="{EMU_W}" cy="{EMU_H}"/></a:xfrm></p:grpSpPr>
      {shapes}
    </p:spTree>
  </p:cSld>
  <p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:sld>
"""


def slide_rels(image_targets: list[str]) -> str:
    rels = [
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>'
    ]
    for i, target in enumerate(image_targets, start=2):
        rels.append(
            f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/{target}"/>'
        )
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="{NS_REL}">{''.join(rels)}</Relationships>
"""


def presentation_xml() -> str:
    ids = "".join(
        f'<p:sldId id="{255 + i}" r:id="rId{i + 1}"/>' for i in range(1, 6)
    )
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:presentation xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
                xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
                xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId1"/></p:sldMasterIdLst>
  <p:sldIdLst>{ids}</p:sldIdLst>
  <p:sldSz cx="{EMU_W}" cy="{EMU_H}" type="wide"/>
  <p:notesSz cx="6858000" cy="9144000"/>
  <p:defaultTextStyle/>
</p:presentation>
"""


def presentation_rels() -> str:
    rels = [
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="slideMasters/slideMaster1.xml"/>'
    ]
    for i in range(1, 6):
        rels.append(
            f'<Relationship Id="rId{i + 1}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" Target="slides/slide{i}.xml"/>'
        )
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="{NS_REL}">{''.join(rels)}</Relationships>
"""


def update_content_types(path: pathlib.Path) -> None:
    ET.register_namespace("", NS_CT)
    tree = ET.parse(path)
    root = tree.getroot()
    for child in list(root):
        if child.tag.endswith("Override") and child.attrib.get("PartName", "").startswith("/ppt/slides/slide"):
            root.remove(child)
    existing_defaults = {c.attrib.get("Extension") for c in root if c.tag.endswith("Default")}
    if "png" not in existing_defaults:
        ET.SubElement(root, f"{{{NS_CT}}}Default", Extension="png", ContentType="image/png")
    for i in range(1, 6):
        ET.SubElement(
            root,
            f"{{{NS_CT}}}Override",
            PartName=f"/ppt/slides/slide{i}.xml",
            ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml",
        )
    tree.write(path, encoding="UTF-8", xml_declaration=True)


def write_app_props(path: pathlib.Path) -> None:
    path.write_text(
        """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties"
            xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">
  <Application>Codex OpenXML generator</Application>
  <Slides>5</Slides>
  <PresentationFormat>Widescreen</PresentationFormat>
</Properties>
""",
        encoding="utf-8",
    )


def write_core_props(path: pathlib.Path) -> None:
    now = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
                   xmlns:dc="http://purl.org/dc/elements/1.1/"
                   xmlns:dcterms="http://purl.org/dc/terms/"
                   xmlns:dcmitype="http://purl.org/dc/dcmitype/"
                   xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>NS-3, Sionna RT, SUMO and TAP Current State</dc:title>
  <dc:creator>Codex</dc:creator>
  <cp:lastModifiedBy>Codex</cp:lastModifiedBy>
  <dcterms:created xsi:type="dcterms:W3CDTF">{now}</dcterms:created>
  <dcterms:modified xsi:type="dcterms:W3CDTF">{now}</dcterms:modified>
</cp:coreProperties>
""",
        encoding="utf-8",
    )


def add_generated_media(work: pathlib.Path, image_map: dict[str, pathlib.Path]) -> None:
    media = work / "ppt" / "media"
    media.mkdir(parents=True, exist_ok=True)
    for target, source in image_map.items():
        shutil.copyfile(source, media / target)


def build_slides(work: pathlib.Path) -> None:
    host = json.loads((RUN / "host_tcp_sink_metadata.json").read_text())
    pcap_size = (RUN / "raw/tcpdump_route.pcap").stat().st_size
    coords = list(csv.DictReader((RUN / "sumo_coordinates.csv").open()))
    heatmaps = sum(
        1
        for d in RUN.glob("coord_*")
        if (d / "raw/sionna_scene.png").exists() and (d / "raw/sionna_grid.png").exists()
    )

    image_map = {
        "deck_architecture_flow.png": ASSET / "architecture_flow.png",
        "deck_signal_map.png": RUN / "plots/trajectory_signal_map.png",
        "deck_sionna_montage.png": ASSET / "sionna_scene_grid_montage.png",
        "deck_channel_triptych.png": ASSET / "trajectory_channel_triptych.png",
        "deck_tcp_delivery.png": ASSET / "tcp_delivery_panel.png",
        "deck_packet_timeline.png": ASSET / "packet_reception_timeline.png",
        "deck_rss_step.png": RUN / "plots/rss_vs_step.png",
        "deck_sinr_step.png": RUN / "plots/sinr_vs_step.png",
    }
    add_generated_media(work, image_map)

    generated_slides = [
        (
            accent_bar()
            + textbox(10, 7, 10, 86, 12, "NS-3 Traffic Through Sionna RT to Linux TAP", 34, True, "1F2937")
            + textbox(11, 7, 26, 82, 8, "Current project state: SUMO mobility, Sionna-managed LTE channel, TCP traffic, TAP delivery, and per-coordinate visualization.", 17, False, "374151")
            + picture(12, "rId2", 6, 39, 88, 42, "architecture_flow.png")
            + textbox(13, 7, 84, 86, 5, "Latest validated run: results/sionna_ns3_experiment_20260507_012424", 11, False, "555555")
            + footer(2),
            ["deck_architecture_flow.png"],
        ),
        (
            accent_bar()
            + textbox(20, 5, 5, 90, 8, "Implemented Workflow", 28, True, "1F2937")
            + picture(21, "rId2", 5, 16, 54, 55, "trajectory_signal_map.png")
            + textbox(22, 62, 17, 32, 48,
                      f"SUMO generated {len(coords)} RX samples for vehicle car_1 on the CNAM route.\n"
                      "ns-3 now runs one continuous trajectory simulation instead of restarting per coordinate.\n"
                      "TCP traffic is generated inside ns-3 and sent through the Sionna-managed LTE path.\n"
                      "TapBridge delivers receiver-side traffic to tap0 in the Linux default namespace.\n"
                      "Sionna heatmaps are exported after the traffic run.", 15, False, "222222")
            + textbox(23, 7, 75, 86, 7, "This plot shows the RX trajectory colored by computed Sionna RSS, so the points are actual evaluated locations.", 12, False, "555555")
            + footer(3),
            ["deck_signal_map.png"],
        ),
        (
            accent_bar()
            + textbox(30, 5, 5, 90, 8, "Channel Metrics And Sionna Outputs", 28, True, "1F2937")
            + picture(31, "rId2", 5, 15, 46, 64, "trajectory_channel_triptych.png")
            + picture(32, "rId3", 53, 15, 42, 41, "sionna_scene_grid_montage.png")
            + textbox(33, 55, 60, 38, 17,
                      "The curves show RSS, path gain, and SINR along the SUMO route.\n"
                      f"The Sionna scene/grid exports confirm route visualization. Heatmaps generated: {heatmaps}/20.", 12, False, "555555")
            + footer(4),
            ["deck_channel_triptych.png", "deck_sionna_montage.png"],
        ),
        (
            accent_bar()
            + textbox(40, 5, 5, 90, 8, "Current Setup And Future Goals", 28, True, "1F2937")
            + textbox(41, 7, 17, 42, 54,
                      "Current setup\n"
                      "ns-3 generates TCP traffic internally.\n"
                      "SUMO provides the moving RX trajectory for car_1.\n"
                      "Sionna RT evaluates channel quality along that route.\n"
                      "Packets exit through EPC/PGW and TapBridge to tap0.\n"
                      "Linux receives the TCP stream and tcpdump records the route traffic.", 16, False, "222222")
            + textbox(42, 53, 17, 40, 54,
                      "Future goals\n"
                      "Tighten per-sample TCP accounting so packet metrics align exactly with each mobility window.\n"
                      "Improve automatic retry for missing Sionna heatmap exports.\n"
                      "Add richer NLOS/path interaction reporting where the Sionna API exposes it.\n"
                      "Move toward live mobility-channel coupling if real-time co-simulation becomes required.", 16, False, "222222")
            + textbox(43, 8, 76, 84, 10,
                      f"Latest proof point: Linux received {int(host.get('received_payload_bytes', 0)):,} TCP payload bytes through tap0; tcpdump captured {pcap_size / 1024:.0f} KB of route traffic.",
                      14, True, "7C2D12")
            + footer(5),
            [],
        ),
    ]

    slides_dir = work / "ppt" / "slides"
    rels_dir = slides_dir / "_rels"
    preserved_slide = (slides_dir / "slide1.xml").read_bytes()
    preserved_rels = (rels_dir / "slide1.xml.rels").read_bytes()
    shutil.rmtree(slides_dir, ignore_errors=True)
    rels_dir.mkdir(parents=True)
    (slides_dir / "slide1.xml").write_bytes(preserved_slide)
    (rels_dir / "slide1.xml.rels").write_bytes(preserved_rels)
    for i, (xml, images) in enumerate(generated_slides, start=2):
        (slides_dir / f"slide{i}.xml").write_text(slide_xml(xml), encoding="utf-8")
        (rels_dir / f"slide{i}.xml.rels").write_text(slide_rels(images), encoding="utf-8")


def zip_directory(source: pathlib.Path, target: pathlib.Path) -> None:
    if target.exists():
        target.unlink()
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as out:
        for path in sorted(source.rglob("*")):
            if path.is_file():
                out.write(path, path.relative_to(source).as_posix())


def main() -> int:
    if not TEMPLATE.exists():
        raise FileNotFoundError(TEMPLATE)
    if not RUN.exists():
        raise FileNotFoundError(RUN)

    with tempfile.TemporaryDirectory(prefix="pptx-build-") as tmp:
        work = pathlib.Path(tmp) / "deck"
        with zipfile.ZipFile(TEMPLATE) as z:
            z.extractall(work)

        build_slides(work)
        (work / "ppt" / "presentation.xml").write_text(presentation_xml(), encoding="utf-8")
        (work / "ppt" / "_rels" / "presentation.xml.rels").write_text(presentation_rels(), encoding="utf-8")
        update_content_types(work / "[Content_Types].xml")
        write_app_props(work / "docProps" / "app.xml")
        write_core_props(work / "docProps" / "core.xml")
        zip_directory(work, OUT)

    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
