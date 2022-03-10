#!/bin/bash
for n in {1..26}
do
  for k in {1..10}
  do
    erasure_code/erasure_code_perf $n  $k
  done
done
