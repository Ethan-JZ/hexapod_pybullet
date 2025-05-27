# A simulation project on moving a hexapod robot

Before actually coding, run the following steps one by one to avoid some unnecessary conflict:

1. Select the folder that you want to save your work: `C:\My_projects\Hexapod` for instance.
2. In your vscode terminal, run: `python -m venv hexapod_env`, there should be a new folder in your current `Hexapod` after this.
3. There is an `activate` file in your `myenv\Scripts`. 
4. Then run: `hexapod_env\Scripts\activate`.
5. Then there should be a `(hexapod_env)` appear in the front of your next line in the terminal.
6. Install your needed library in the vscode terminal: `pip install numpy`, `pip install numpy`, `pip install pybullet` (BUT for pybullet, see next line!!!!).
7. Before installing `pybullet`, make sure you have installed visual studio developer from https://visualstudio.microsoft.com/vs/professional/.

This is a project on moving a hexapod robot. Folder of `real_world_control` is to control robot in real-world. The author uses Hiwonder HTD-45 servo in the CAD design. The library to control HTD-45 servo is from https://github.com/ethanlipson/PyLX-16A. I find it surprised that the code to control LX-16 servo from Hiwonder can still work on HTD-45 servo. (^_^).

