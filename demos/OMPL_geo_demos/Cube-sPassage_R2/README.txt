To launch the benchmarkings:

1) Open a terminal in the directory where the kautham-console application is:

2) Launch de benchmarking using the kautham-console application (the argument is the absolute path of the benchmarking file):

    > ./kautham-console -b abs_path_demo_folder_xml_benchmarking_file [optional:abs_path_models_folder]

3) Copy the python file kautham/python/kautham_ompl_benchmark_statistics.py into the demo folder.

4) Generate the database using the python file:

    > ./kautham_ompl_benchmark_statistics.py resultPRM.log -d resultPRM.db
    
4) View the results:

   Upload the database file at plannerarena.org


