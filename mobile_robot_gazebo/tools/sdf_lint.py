#!/usr/bin/env python3
"""Check SDF files against the official sdformat schema (catches misspelled or misplaced tags that Gazebo silently ignores).

   git clone --depth 1 --filter=blob:none --sparse https://github.com/gazebosim/sdformat /tmp/sdformat
   git -C /tmp/sdformat sparse-checkout set sdf/1.9
   python3 mobile_robot_gazebo/tools/sdf_lint.py /tmp/sdformat/sdf/1.9 mobile_robot_gazebo/worlds/*.sdf mobile_robot_gazebo/models/*/model.sdf

Exit code 1 when a file has a tag or attribute the schema does not allow. <plugin> contents are free-form, as in Gazebo.
Pure Python (xml.etree). Also run by the unit tests when the environment variable SDFORMAT_SDF_DIR points at the schema."""
import glob
import os
import sys
import xml.etree.ElementTree as ET


def load_schema(directory):
    """file name (without .sdf) -> root <element>. Keyed by FILE name: several files define a root called 'model'."""
    return {os.path.splitext(os.path.basename(f))[0]: ET.parse(f).getroot() for f in glob.glob(os.path.join(directory, "*.sdf"))}


def _children(spec_el, schema):
    names, attrs = {}, set()
    for c in spec_el:
        if c.tag == "element":
            names[c.get("name")] = schema.get(c.get("ref")) if c.get("ref") else c
        elif c.tag == "include":
            stem = os.path.splitext(c.get("filename"))[0]
            root = schema.get(stem)
            names[root.get("name") if root is not None else stem] = root
        elif c.tag == "attribute":
            attrs.add(c.get("name"))
    if spec_el.get("copy_data") == "true":
        names["*"] = None
    return names, attrs


def lint(path, schema):
    problems = []

    def walk(node, spec_el, trail):
        names, attrs = _children(spec_el, schema)
        for a in node.attrib:
            if a not in attrs:
                problems.append(f"{trail}: unknown attribute '{a}'")
        for child in node:
            if child.tag is ET.Comment or "*" in names or node.tag == "plugin":
                continue
            if child.tag not in names:
                problems.append(f"{trail}/{child.tag}: not allowed here")
                continue
            sub = names[child.tag] if names[child.tag] is not None else schema.get(child.tag)
            if sub is not None:
                label = f"{child.tag}[{child.get('name')}]" if child.get("name") else child.tag
                walk(child, sub, f"{trail}/{label}")

    walk(ET.parse(path).getroot(), schema["root"], "sdf")
    return problems


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    schema = load_schema(argv[1])
    bad = 0
    for f in argv[2:]:
        problems = lint(f, schema)
        bad += len(problems)
        print(f"{'OK  ' if not problems else 'FAIL'} {f}" + ("" if not problems else f"  ({len(problems)} problems)"))
        for p in problems[:10]:
            print("      " + p)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
