#!/bin/bash
for n in {1..50}
do
  for k in {1..10}
  do
    erasure_code/erasure_code_perf_real_chunksize $n  $k 128
  done
done
