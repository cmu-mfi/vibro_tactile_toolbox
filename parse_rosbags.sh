for f in results/2by1/*.bag; do
  echo $f
  python scripts/parse_rosbag.py -b $f -d data/2by1/
done
for f in results/2by2/*.bag; do
  echo $f
  python scripts/parse_rosbag.py -b $f -d data/2by2/
done
for f in results/2by4/*.bag; do
  echo $f
  python scripts/parse_rosbag.py -b $f -d data/2by4/
done
for f in results/4by1/*.bag; do
  echo $f
  python scripts/parse_rosbag.py -b $f -d data/4by1/
done
for f in results/4by2/*.bag; do
  echo $f
  python scripts/parse_rosbag.py -b $f -d data/4by2/
done