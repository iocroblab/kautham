# Kautham Benchmarking Guide

This guide explains how to run benchmarking experiments using the **`kautham-console`** application and analyze the results with [Planner Arena](http://plannerarena.org/).

---


## 🚀 Running the Benchmarking

1. **Open a terminal** in the directory where the `kautham-console` binary is located.

2. **Run the benchmarking** with your XML configuration file:

```
./kautham-console -b abs_path_demo_folder_xml_benchmarking_file [optional:abs_path_models_folder]
```

- `<ABS_PATH_TO_BENCHMARK_FILE>`: Full path to the benchmarking XML file  
- `[ABS_PATH_TO_MODELS_FOLDER]`: *(Optional)* Path to the models folder

**Example:**
```
./kautham-console -b ~/kautham/demos/benchmarking/demo.xml ~/kautham/models
```


---

## 📊 Processing Results

3. **Copy the Python statistics script** into your demo folder:

```
cp kautham/python/kautham_ompl_benchmark_statistics.py <YOUR_DEMO_FOLDER>
```


4. **Generate a database** from the raw log file:
```
./kautham_ompl_benchmark_statistics.py resultPRM.log -d resultPRM.db
```

This will create a SQLite database file (`resultPRM.db`) containing your benchmarking results.

---

## 🌐 Visualizing Results

5. **Upload the database** to [Planner Arena](http://plannerarena.org/):

- Open the website  
- Upload `resultPRM.db`  
- Explore and compare planner performance interactively  


---

## ✅ Notes & Tips
- Ensure the log file (`resultPRM.log`) is created after running the benchmark.  
- Use descriptive names for log and database files if running multiple experiments.  
- Consider storing multiple `.db` files to compare planners across benchmarks.  

---

✨ You’re now ready to run benchmarks, process results, and visualize them with **Kautham + OMPL + Planner Arena**!

