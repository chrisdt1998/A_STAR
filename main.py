"""
This is the A* algorithm project which is visualized using Kivy. This project was designed and written by Christopher
du Toit with the intention to understand the A* algorithm and gain some basic understanding of Kivy.
"""


from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.gridlayout import GridLayout
from kivy.animation import Animation
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen

from astarsearch import AStarSearch



class StartWindow(Screen):
    pass


class SearchWindow(Screen):
    """
    This is a class which inherits the GridLayout class from Kivy. The idea for this class is to visualize the A*
    algorithm using Kivy.
    """
    def __init__(self, **kwargs):
        """
        Initializing the class.
        """
        # super(SearchWindow, self).__init__(**kwargs)
        super().__init__(**kwargs)
        self.include_diag = True
        self.reset_grid(None)

    def reset_grid(self, instance):
        self.clear_widgets()
        self.main_grid = FloatLayout()
        self.counter = 0
        self.iter = 0
        self.start = []
        self.end = []
        self.walls = []
        self.visualized_nodes = []
        self.all_nodes = []
        self.shortest_path = []
        self.speed = 60
        self.num_cols = 15
        self.num_rows = 30
        self.b = []
        self.search_grid = GridLayout(size_hint=(0.9, 0.9),
                                        pos_hint={"x": 0.05, "top": 1})
        self.search_grid.cols = self.num_cols

        for i in range(self.num_cols * self.num_rows):
            self.b.append(CustomButton())
            # self.b[i].bind(on_press=self.button_clicked)
            self.b[i].id_num = i
            self.search_grid.add_widget(self.b[i])
        self.main_grid.add_widget(self.search_grid)

        self.include_diags_button = Button(text=f"Include Diag: {self.include_diag}", font_size=20, size_hint=(0.3, 0.075), pos_hint={'x':0.03, 'top':0.09})
        self.include_diags_button.bind(on_press=self.include_diag_toggle)
        self.main_grid.add_widget(self.include_diags_button)

        self.start_button = Button(text="Start search", font_size=20, size_hint=(0.3, 0.075), pos_hint={'x':0.35, 'top':0.09})
        self.start_button.bind(on_press=self.start_search)
        self.main_grid.add_widget(self.start_button)

        self.reset_button = Button(text="Reset grid", font_size=20, size_hint=(0.3, 0.075), pos_hint={'x':0.68, 'top':0.09})
        self.reset_button.bind(on_press=self.reset_grid)
        self.main_grid.add_widget(self.reset_button)

        self.add_widget(self.main_grid)

    def include_diag_toggle(self, button):
        self.include_diag = not self.include_diag
        button.text = f"Include Diag: {self.include_diag}"

    def start_search(self, instance):
        if self.start and self.end:
            print("Start running")
            astar = AStarSearch(self.start, self.end, self.walls, self.num_rows, self.num_cols, self.include_diag)
            astar.search()
            if astar.failed is False:
                self.shortest_path = astar.backtrack()
                print(len(self.shortest_path))
            self.all_nodes = astar.stored_nodes
            Clock.schedule_interval(self.animate, 1 / self.speed)
            print("Visualization finished")
        else:
            print("Choose a starting and ending point first!")

    def animate(self, dt):
        """
        This method is responsible for the animations of the buttons for when the searching is happening and the
        backtracking.
        :param dt: change in time. Parameter mandatory for Kivy but not used.
        :type dt: None
        """
        if self.iter < len(self.all_nodes):
            index = (self.all_nodes[self.iter][0] * self.num_cols) + self.all_nodes[self.iter][1]
            self.b[index].background_normal = ''
            if self.all_nodes[self.iter] not in self.visualized_nodes:
                if self.all_nodes[self.iter] != self.end and self.all_nodes[self.iter] != self.start:
                    anim = Animation(background_color=(0, 0, 1, .85))
                    anim.start(self.b[index])
                    self.visualized_nodes.append(self.all_nodes[self.iter])
            else:
                anim = Animation(background_color=(0.5, 0, 0.5, .85))
                anim.start(self.b[index])
        elif self.iter >= len(self.all_nodes) and self.iter < len(self.all_nodes) + len(self.shortest_path) and len(self.shortest_path) > 0:
            i = self.iter - len(self.all_nodes)
            index = (self.shortest_path[i][0] * self.num_cols) + self.shortest_path[i][1]
            if self.shortest_path[i] != self.end and self.shortest_path[i] != self.start:
                self.b[index].background_normal = ''
                anim = Animation(background_color=(1, 215/255, 0, 1))
                anim.start(self.b[index])
        else:
            return False
        self.iter += 1

    def button_clicked(self, button):
        """
        This method is called when a button on the Kivy UI is clicked. The first button click is the starting node,
        the second button click is the ending node and the following 12 buttons clicked are the walls. After that, the
        searching algorithm automatically starts.
        :param button: Button clicked
        :type button: Kivy Widget
        """
        if self.counter == 0:
            button.background_normal = ''
            anim = Animation(background_color=(0, 1, 0, .85))
            anim.start(button)
            self.start = [int(button.id_num / self.num_cols), int(button.id_num % self.num_cols)]
            print(f"Starting position {self.start}")
        elif self.counter == 1:
            button.background_normal = ''
            anim = Animation(background_color=(1, .3, .4, .85))
            anim.start(button)
            self.end = [int(button.id_num / self.num_cols), int(button.id_num % self.num_cols)]
            print(f"Ending position {self.end}")
        else:
            anim = Animation(background_color=(0, 0, 0, .85))
            anim.start(button)
            self.walls.append([int(button.id_num / self.num_cols), int(button.id_num % self.num_cols)])

        self.counter += 1


class WindowManager(ScreenManager):
    pass

class CustomButton(Button):
    def __init__(self, **kwargs):
        super(CustomButton, self).__init__(**kwargs)
        self.counter = 0

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            app.root.ids.searchwindow.button_clicked(self)

    def on_touch_move(self, touch):
        self.on_touch_down(touch)


class AStarApp(App):
    """
    This class represents the Kivy Application UI. Is only used to created the grid, can be used for further work such
    as creating new windows etc.
    """

    def build(self):
        kv = Builder.load_file("AStar.kv")
        return kv

if __name__ == '__main__':
    app = AStarApp()
    app.run()


