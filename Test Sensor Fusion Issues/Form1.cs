using ArtemisCore.Calculations;
using System;
using System.Runtime.InteropServices;
using System.Windows.Forms;


namespace Test_Sensor_Fusion_Issues
{
  public partial class Form1 : Form
  {
    // Import the AllocConsole method from kernel32.dll
    [DllImport("kernel32.dll")]
    private static extern bool AllocConsole();

    // Import the FreeConsole method to close the console
    [DllImport("kernel32.dll")]
    private static extern bool FreeConsole();

    // Import the SetWindowPos method to move the console window
    [DllImport("user32.dll", SetLastError = true)]
    private static extern bool SetWindowPos(IntPtr hWnd, IntPtr hWndInsertAfter, int X, int Y, int cx, int cy, uint uFlags);

    // Get the handle of the console window
    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern IntPtr GetConsoleWindow();

    public Form1()
    {
      InitializeComponent();
    }

    private void Form1_Load(object sender, EventArgs e)
    {
      // Launch and position the console
      LaunchConsole();
      PositionConsole();

      // Position the form in the upper-right quarter
      PositionForm();

      Solver.Solve(@"C:\LocalDev\testSensorFusion\Data.csv");

    }



    private void LaunchConsole()
    {
      AllocConsole();
      Console.WriteLine("Console launched!");
      Console.WriteLine("You can use this console to debug or log data.");
    }

    private void PositionConsole()
    {
      IntPtr consoleHandle = GetConsoleWindow();
      if (consoleHandle == IntPtr.Zero)
        return;

      // Get the screen dimensions
      int screenWidth = Screen.PrimaryScreen.Bounds.Width;
      int screenHeight = Screen.PrimaryScreen.Bounds.Height;

      // Calculate the size and position for the upper-left quarter
      int width = screenWidth / 2;
      int height = screenHeight / 2;
      int x = 0;
      int y = 0;

      // Set the console window position and size
      SetWindowPos(consoleHandle, IntPtr.Zero, x, y, width, height, 0);
    }

    private void PositionForm()
    {
      // Get the screen dimensions
      int screenWidth = Screen.PrimaryScreen.Bounds.Width;
      int screenHeight = Screen.PrimaryScreen.Bounds.Height;

      // Calculate the size and position for the upper-right quarter
      int width = screenWidth / 2;
      int height = screenHeight / 2;
      int x = screenWidth / 2;
      int y = 0;

      // Set the form size and location
      this.Size = new System.Drawing.Size(width, height);
      this.Location = new System.Drawing.Point(x, y);
    }

    private void CloseConsole()
    {
      FreeConsole();
    }
  }
}
