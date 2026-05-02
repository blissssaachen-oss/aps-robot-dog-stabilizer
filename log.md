## How data was collected
The follow command was ran for the collecting data

```bash
python3 posture_stablization.py 2>&1 | tee "<name of log file>.log"
```

## Data conversion from log to csv
The following command was ran for converting log to csv

```bash
python3 log_to_csv.py <input>.log <output>.csv
```
