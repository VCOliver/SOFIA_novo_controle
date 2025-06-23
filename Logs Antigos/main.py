from typing import Final
from pathlib import Path
import pandas as pd

POTENCIA_ALVO: Final = 5
FOLDER_PATH: Path = Path("combined.csv")

df = pd.read_csv(FOLDER_PATH)

inputs_df = df[['Segundo', 'Corrente', 'Impedancia']]

inputs_df.to_csv('simul_input.csv', index=False)
