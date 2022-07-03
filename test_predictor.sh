for i in testcases/*.data; do
  name=$(basename "$i" .data)
  output=$(./code <$i 2>&1 >/dev/null)
  echo "|$name|$output|" >>predict_res.txt
done
