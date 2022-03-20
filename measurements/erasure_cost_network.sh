#!/bin/bash
for n in {1..8}
do
  for k in {1..8}
  do
    erasure_code/erasure_code_perf_chunksize $n  $k $((128*$n))
  done
done
