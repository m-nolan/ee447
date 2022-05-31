# ee447.py
## Code repository for Control Systems I (EE447)
Sep Makhsous, Michael Nolan

##### Installation:

clone the git repo from github

`git clone https://github.com/m-nolan/ee447.git`

to install the necessary packages, install the package using `pip`:
`cd ~/your/path/to/repo/ee447`
`pip install -e .`

##### To run:
`cd ./ee447`
`python ./pidopt.py`

Change around the parameters in the `main()` method and the script constants at the top to define the optimization.

##### Packages:

- pidopt.py
    - PID controller parameter optimization script