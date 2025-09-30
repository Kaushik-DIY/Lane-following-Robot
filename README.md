# Getting Started

## Linux Systems installation steps

```BASH
git clone https://github.com/EmbeddedSystems-UniversitySiegen/eclab-task3.git
cd eclab-task3
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Windows Systems installation steps

```BASH
Open Command Prompt (or PowerShell)
git clone https://github.com/EmbeddedSystems-UniversitySiegen/eclab-task3.git
cd eclab-task3
py -m venv venv
.\venv\Scripts\activate.bat 
pip install -r .\requirements.txt

```
## Run the file in the activated virtual environment

```BASH
Open Command Prompt (or PowerShell)
cd src
python ./main.py
```
This will connect the python script to coppelia simulator.

Refer to the task documentation for further details. 

## Tests

Install pytest as follows in the virtual environment created above.

```BASH
pip install pytest
```

Run tests using pytest.

``` BASH
pytest
```
