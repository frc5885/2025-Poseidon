# Setup

## Create virtual environment
Creates a python virtual environment in a folder called "env" where we will install the requirements

```
cd python-operator-panel
python -m venv env
```

## Activate the virtual environment
Do this before running "pip install" so that the packages get installed into the virtual environment.

```
.\env\Scripts\activate
```

If you get an error about "running scripts is disabled on this system", open PowerShell as an administrator and run `Set-ExecutionPolicy -ExecutionPolicy Unrestricted`


## Install the requirements

```
pip install -r requirements.txt
```

## Set up VSCode to use the virtual environment
For VSCode to know that the packages are installed in the virtual environment and not give errors, we need to change the path to the python .exe file it uses.

 - Open main.py
 - Click the python version in the bottom right
 - Click "Enter Interpreter Path"
 - Enter `.\python-operator-panel\env\Scripts\python.exe`
 - Now all the error/warning underlines should go away

## Run the app

```
python main.py
```

OR

Click the **Play** button in the top right
