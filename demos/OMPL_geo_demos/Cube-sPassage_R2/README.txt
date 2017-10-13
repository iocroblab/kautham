To launch the benchmarkings:

1) Open a terminal in the directory where the kautham-console application is:

2) Launch de benchmarking using the kauthma_console application:

    > ./kautham-console -b abs_path_xml_benchmarking_file [optional:abs_path_models_folder]

3) Generate the database:

    > ompl_benchmark_statistics resultPRM.log -d resultPRM.db
    
4) View the results:

   a) Generate the plots in pdf:

       > ompl_benchmark_statistics -d resultPRM.db -p resultPRM.pdf
 
   b) upload the database file at plannerarena.org


