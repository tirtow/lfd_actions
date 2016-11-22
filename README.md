FRI Project to classify actions based on demonstration from a person

Recording data:
- `rosrun lfd_action knn [-h]` will display the avaliable options
- `rosrun lfd_actions knn -d <source file>` will load the specified dataset
- `rosrun lfd_actions knn -d <source file> -s` will save to the specified dataset
- `rosrun lfd_actions knn -d <source file> -t <file>` will load the specified dataset and run the specified test file
- `rosrun lfd_actions knn -d <source file> -v` will load the specified dataset and run in verbose mode
- `rosrun lfd_actions knn -d <source file> -k <int>` will load the specified dataset and run with k neighbors
