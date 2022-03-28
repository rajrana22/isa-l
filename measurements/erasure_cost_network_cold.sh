#!/bin/bash
for n in {16..16}
do
  for k in {1..8}
  do
    erasure_code/erasure_code_perf_chunksize_cold $n  $k $((128*$n))
  done
done
