import pandas as pd
import sys

if len(sys.argv) < 3:
    print("Uso: python export_reference_from_csv.py input.csv output_reference.csv")
    sys.exit(1)

inp = sys.argv[1]
out = sys.argv[2]

df = pd.read_csv(inp)

# Tomamos solo t y las columnas qref[...]
cols = ["t"] + [c for c in df.columns if c.startswith("qref[")]
if len(cols) == 1:
    raise SystemExit("No se encontraron columnas qref[...] en el CSV: " + inp)

df2 = df[cols].copy()
# Renombrar qref[joint] -> joint
df2.columns = ["t"] + [c[5:-1] for c in cols[1:]]

df2.to_csv(out, index=False)
print("Guardado:", out)
