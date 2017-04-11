#!/bin/bash

for level in levels/*
    do
    for algo in {0..3}
    do
        for t in {1..5}
        do
            python3 main.py -i$level -o./output/algo$algo$(basename $level .json)try$t.json -a$algo
        done
    done
done