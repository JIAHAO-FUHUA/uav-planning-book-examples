#!/usr/bin/env bash
# Ubuntu 20.04 / ROS Noetic. No sudo; prerequisites must already be installed.
set -eo pipefail
source /opt/ros/noetic/setup.bash
set -u
book_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
book_root="${UAV_BOOK_CASE_ROOT:-$HOME/uav_book_cases_20261020}"
mkdir -p "$book_root"
book_root="$(cd "$book_root" && pwd)"
python3 "$book_script_dir/prepare_sources.py" --root "$book_root"
python3 - "$book_root" <<'PY'
from pathlib import Path
import hashlib,tarfile,urllib.request,sys
r=Path(sys.argv[1]);archive=r/'nlopt_v2.7.1.tar.gz'
expected='db88232fa5cef0ff6e39943fc63ab6074208831dc0031cf1545f6ecd31ae2a1a'
if not archive.exists():
    with urllib.request.urlopen('https://codeload.github.com/stevengj/nlopt/tar.gz/refs/tags/v2.7.1',timeout=120) as f:archive.write_bytes(f.read())
assert hashlib.sha256(archive.read_bytes()).hexdigest()==expected,'NLopt archive hash mismatch'
if not (r/'nlopt-2.7.1/CMakeLists.txt').exists():
    with tarfile.open(archive) as t:
        for member in t.getmembers():
            target=(r/member.name).resolve()
            assert str(target).startswith(str(r.resolve())+'/'),member.name
            assert not member.issym() and not member.islnk(),member.name
        t.extractall(r)
p=r/'fast_ws/src/fast_planner/bspline_opt/CMakeLists.txt'
s=p.read_text().replace('/usr/local/include',str(r/'deps/include')).replace('/usr/local/lib/libnlopt.so',str(r/'deps/lib/libnlopt.so'));p.write_text(s)
PY
if [ ! -f "$book_root/deps/lib/libnlopt.so" ]; then
  cmake -S "$book_root/nlopt-2.7.1" -B "$book_root/nlopt-build" \
    -DCMAKE_INSTALL_PREFIX="$book_root/deps" -DCMAKE_INSTALL_LIBDIR=lib \
    -DNLOPT_PYTHON=OFF -DNLOPT_OCTAVE=OFF -DNLOPT_MATLAB=OFF -DNLOPT_GUILE=OFF -DNLOPT_SWIG=OFF
  cmake --build "$book_root/nlopt-build" -j4 > "$book_root/nlopt_build.log" 2>&1
  cmake --install "$book_root/nlopt-build" >> "$book_root/nlopt_build.log" 2>&1
fi
export LD_LIBRARY_PATH="$book_root/deps/lib:${LD_LIBRARY_PATH:-}"
for book_kind in fast ego; do
  (cd "$book_root/${book_kind}_ws"; catkin_make -j4 -l4 -DCMAKE_BUILD_TYPE=Release > "$book_root/${book_kind}_build.log" 2>&1)
  # Save an environment helper; a fresh shell must source only one workspace.
  printf 'source /opt/ros/noetic/setup.bash\nsource %q\nexport LD_LIBRARY_PATH=%q:"${LD_LIBRARY_PATH:-}"\n' \
    "$book_root/${book_kind}_ws/devel/setup.bash" "$book_root/deps/lib" > "$book_root/case_${book_kind}_env.sh"
  printf '%s BUILD PASSED\n' "$book_kind"
done
python3 "$book_script_dir/make_launch.py" --root "$book_root"
