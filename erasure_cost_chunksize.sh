#!/bin/bash


benchmark(){
  n=$1
  k=$2
  for chunksize in {128,256,512,1024,1024*2,1024*4,1024*8,1024*16,1024*32}
  do
    erasure_code/erasure_code_perf_chunksize $n  $k $chunksize
  done
}

benchmark 8 1
benchmark 8 2
benchmark 8 3
benchmark 8 4
benchmark 4 1
benchmark 4 2
benchmark 16 1
benchmark 16 2
benchmark 16 3
benchmark 16 4
benchmark 16 5
benchmark 16 6
benchmark 16 7
benchmark 16 8
