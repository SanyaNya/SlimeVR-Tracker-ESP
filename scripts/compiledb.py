import os
import subprocess

#Prevent recursion
if not os.environ.get("COMPILEDB"):
    compiledb_env = os.environ.copy()
    compiledb_env["COMPILEDB"] = "1"

    #Run compiledb
    subprocess.run(["pio", "run", '-t', "compiledb"], input='\n', encoding='ascii', stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, env=compiledb_env)
