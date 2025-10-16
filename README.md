# MultiAxisController_MG43

- MG43电机多轴运动系统
    - mas04: 12dof四足狗

## run

```sh
# python3.10

conda create -n doglab python=3.10
conda activate doglab
conda env export > environment.yml
conda env create -f environment.yml

pip install -r requirements.txt
pip freeze > requirements.txt

python ./scripts/run_ik.py
python ./scripts/run_rl.py
```


