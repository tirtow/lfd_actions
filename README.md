FRI Project to classify actions based on demonstration from a person

Recording data:
- General use:
    - `rosrun lfd_actions knn -d <file> [0hvcs] [-k <int>] [-t <file>]`

- Example uses:
    - `rosrun lfd_action knn -h` will display the avaliable options
    - `rosrun lfd_actions knn -d <file>` will load the specified dataset
    - `rosrun lfd_actions knn -d <file> -s` will save to the specified dataset
    - `rosrun lfd_actions knn -d <file> -t <file>` will load the specified dataset and run the specified test file
    - `rosrun lfd_actions knn -d <file> -v` will load the specified dataset and run in verbose mode
    - `rosrun lfd_actions knn -d <file> -k <int>` will load the specified dataset and run with k neighbors
