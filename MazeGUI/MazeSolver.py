import MazeEnv
from AlgorithmAgents import *
import time
from IPython.display import clear_output
import defs
import TranslateToRobotActions
import tkinter as tk
import MazeBuilder
import gspread
from oauth2client.service_account import ServiceAccountCredentials


def mapMzeSymbolsToLetters(maze):
    symbol_to_letter = {
        'W': 'W',  # Wall
        'F': 'F',  # Free space
        'S': 'S',  # Start point
        'G': 'G'   # Goal
    }
    return [[symbol_to_letter[cell] for cell in row] for row in maze]


def transmitMaze(maze):
    """
    Transmit the maze structure to Google Sheets.
    Each row of the maze will be written as a row in the Google Sheet.
    """
    scope = ["https://spreadsheets.google.com/feeds", "https://www.googleapis.com/auth/drive"]
    creds = ServiceAccountCredentials.from_json_keyfile_name("JSONkey.json", scope)
    client = gspread.authorize(creds)
    sheet = client.open("Transmit").sheet1  # Open the first sheet

    # Clear the existing data in the sheet
    sheet.clear()

    # Write the maze structure row by row
    for row in maze:
        sheet.append_row(row)


def buildMaze(size, print_maze=0):
    """
    Build a maze using MazeBuilder and store it in defs.boards['Map'].
    """
    root_builder = tk.Tk()
    MazeBuilder.MazeBuilder(root_builder, rows_builder=size, cols_builder=size)  # Adjust size as needed
    root_builder.mainloop()
    if print_maze:
        env = MazeEnv(defs.boards['Map'])
        env.reset()
        print(env.render())
        clear_output(wait=True)

    # Return the built maze
    return defs.boards['Map']


# Main execution
if __name__ == "__main__":
    raw_maze = buildMaze(10, print_maze=1)  # Create and retrieve the raw maze
    leteteral_maze = mapMzeSymbolsToLetters(raw_maze)  # Map symbols to numerical values
    transmitMaze(leteteral_maze)  # Transmit the numerical maze to Google Sheets
