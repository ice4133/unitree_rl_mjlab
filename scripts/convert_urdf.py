"""Convert a URDF and its mesh references to MJCF using MuJoCo's importer."""

import argparse
import os
from pathlib import Path
import sys
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

import mujoco


def resolve_asset(
  filename: str, urdf: Path, asset_dir: Path, package_root: Path | None
) -> Path:
  """Resolve local paths and ROS package URIs without requiring ROS."""
  uri = urlsplit(filename)
  if uri.scheme == "package":
    # Some SolidWorks exports use '#' in package names. Parse package://
    # directly so those characters remain part of the local asset path.
    package, separator, relative = filename[len("package://"):].partition("/")
    package, relative = unquote(package), unquote(relative)
    if not package or not separator or not relative:
      raise ValueError(f"Invalid package asset path: {filename}")
    candidates = []
    if package_root is not None:
      candidates.extend((package_root / relative, package_root / package / relative))
    for parent in urdf.parents:
      manifest = parent / "package.xml"
      package_name = ET.parse(manifest).findtext("name") if manifest.is_file() else None
      if parent.name == package or package_name == package:
        candidates.append(parent / relative)
      candidates.append(parent / package / relative)
    hint = " Supply --package-root with the ROS package directory if needed."
  elif uri.scheme == "file":
    if uri.netloc not in ("", "localhost"):
      raise ValueError(f"Non-local file URI is not supported: {filename}")
    candidates = [Path(unquote(uri.path))]
    hint = ""
  elif not uri.scheme:
    candidates = [asset_dir / filename]
    hint = " Paths must be relative to the URDF (or its compiler meshdir)."
  else:
    raise ValueError(f"Unsupported asset URI: {filename}")

  for candidate in candidates:
    if candidate.is_file():
      return candidate.resolve()
  raise FileNotFoundError(f"Cannot find asset: {filename}.{hint}")


def convert_urdf(urdf: Path, output: Path, package_root: Path | None = None):
  urdf = urdf.expanduser().resolve()
  output = output.expanduser().resolve()
  if not urdf.is_file():
    raise FileNotFoundError(f"URDF does not exist: {urdf}")
  if output.exists():
    raise FileExistsError(f"Output already exists; choose another --output: {output}")
  if package_root is not None:
    package_root = package_root.expanduser().resolve()
    if not package_root.is_dir():
      raise NotADirectoryError(f"Package directory does not exist: {package_root}")

  root = ET.parse(urdf).getroot()
  if root.tag != "robot":
    raise ValueError("Expected a plain URDF with a <robot> root element.")
  if any("xacro" in element.tag for element in root.iter()):
    raise ValueError("Expand xacro to a plain URDF before converting.")

  extension = root.find("mujoco")
  if extension is None:
    extension = ET.SubElement(root, "mujoco")
  compiler = extension.find("compiler")
  if compiler is None:
    compiler = ET.SubElement(extension, "compiler")
  mesh_dir = urdf.parent / compiler.get("meshdir", compiler.get("assetdir", ""))
  texture_dir = urdf.parent / compiler.get("texturedir", compiler.get("assetdir", ""))
  for tag, directory in (("mesh", mesh_dir), ("texture", texture_dir)):
    for asset in root.iter(tag):
      filename = asset.get("filename")
      if filename:
        asset.set("filename", str(resolve_asset(filename, urdf, directory, package_root)))

  # Preserve visuals and fixed links for inspecting the imported assembly.
  # The original URDF is untouched; MuJoCo receives this in-memory copy.
  compiler.set("discardvisual", "false")
  compiler.set("fusestatic", "false")
  compiler.set("strippath", "false")
  for key in ("assetdir", "meshdir", "texturedir"):
    compiler.attrib.pop(key, None)
  spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
  model = spec.compile()
  mjcf = ET.fromstring(spec.to_xml())

  # Keep asset references relative to the output, so the bundle can be moved.
  for asset in mjcf.findall("./asset/*"):
    filename = asset.get("file")
    if filename and Path(filename).is_absolute():
      asset.set("file", os.path.relpath(filename, output.parent))
  ET.indent(mjcf, space="  ")
  output.parent.mkdir(parents=True, exist_ok=True)
  with output.open("x", encoding="utf-8") as stream:
    stream.write(ET.tostring(mjcf, encoding="unicode") + "\n")
  try:
    mujoco.MjModel.from_xml_path(str(output))
  except Exception:
    output.unlink()
    raise
  return model


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("urdf", type=Path, help="Path to the input URDF")
  parser.add_argument(
    "--output", type=Path, help="Output MJCF path (default: next to URDF, .xml extension)"
  )
  parser.add_argument(
    "--package-root", type=Path,
    help="ROS package directory containing meshes, for package:// references",
  )
  args = parser.parse_args()
  output = args.output or args.urdf.with_suffix(".xml")
  try:
    model = convert_urdf(args.urdf, output, args.package_root)
  except (OSError, ValueError, ET.ParseError) as error:
    print(f"Conversion failed: {error}", file=sys.stderr)
    return 1
  print(f"MJCF saved and reloaded successfully: {output.expanduser().resolve()}")
  print(f"Robot bodies: {model.nbody - 1}; joints: {model.njnt}; meshes: {model.nmesh}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
