#!/bin/zsh
# conda init zsh
# conda activate gridworld
for mazenum in {0..100}
do 
    python forward_repeated_a_star.py $mazenum
done