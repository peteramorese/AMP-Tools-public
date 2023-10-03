# Workspace
This is the place (and only place) that you will put all of your code. To help you structure your workspace, you can create directories inside `ws/`for each executable you would like to make. For example, you will have a `main()` executable for each assignment, so you can create a separate directory for each assignment.

## Navigation
Put all of your projects inside directories in this directory (i.e. `ws/hw2`, `ws/my_project`, etc.). If you have classes that you would like to use across multiple projects/executables (e.g. collision checking perhaps?), you can put them in a directory named `shared` (`ws/shared`) and they will automatically be linked to all of your executables.

## Making a project
Make a new direcotry `my_project` inside this `ws/` directory. Inside `my_project`, make sure you have a `.cpp` file that has `main()`. 

In order to run your project, pass the name of your project (same name as the directory) to `build_and_run.sh`:
```
bash build_and_run.sh my_project <any-flags-here>
```

In order to grade your project, pass the name of your project (same name as the directory) to `grade.sh`:
```
bash grade.sh my_project --hw2
```
This command will grade the executable in `my_project` against `hw2`. If you decide to make your projects after each homework:
```
bash grade.sh hw2 --hw2
```


