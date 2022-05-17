# **A Star searching with Kivy Visualizations**

## **Introduction:**

This repository contains a project where A* searching algorithm is implemented and visualized using the Kivy GUI.

## **Learning goals and background:**

The learning goals for this project was to understand and learn how to implement the A* searching algorithm. The A* 
algorithm essentially works by minimizing a function f(n) = g(n) + h(n) where g(n) represents the cost function to the
node n and h(n) is the heuristic of the node n. Moreover, in this project, the distances are computed using Manhattan 
distance and so g(n) is basically the number of nodes that it travelled through to get to node n and h(n) is the Manhattan
distance from the current node to the final goal node. 

The other main learning objective of this project was to use and understand Kivy for the first time. Kivy is more 
powerful than Pygame when it comes to GUI and simple game development but is also much more complicated to use due to
the class inheritance. A main desirable feature of Kivy is how easy it is to convert the application into an android 
application.

## **Code structure:**

- main.py contains all the necessary code for the A* search and the Kivy application.
- Astar.kv contains the kivy file for the App. This is still a work in progress.


## **Current Progress:**

- The A* searching algorithm is completed. The A* searching algorithm was rewritten almost entirely, and it now works
much better with no (known) bugs.
- The Kivy GUI is functional but there is still a lot of potential work to be done. For instance, creating multiple
windows such as an intro window and an ending window. 
- I also want to introduce new features such as being able to change the include_diag boolean within the GUI and also
having a flexible number of walls and a start button. 