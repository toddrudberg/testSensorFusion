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
            cLHT FRSXform = new cLHT(0,0,0,0,0,2 * Math.PI / 180.0);
            cPose testzyx = new cPose(0,0,0,-30,15,-15);
            cLHT testXform = new cLHT();
            testXform.setTransformFromEulerZYX(testzyx);
            cPose testxyz = testXform.getPoseEulerXYZ();


            FRSXform = new cLHT();
            for (int rz = -15; rz <= 15; rz+=15)
            {
                for( int ry = -15; ry <= 15; ry+=15)
                {
                    for( int rx = -15; rx <= 15; rx+=15)
                    {
                        cLHT eulerzyx = new cLHT();
                        cPose peulerzyx = new cPose(0,0,0,rx,ry,rz);
                        eulerzyx.setTransformFromEulerZYX(peulerzyx);
                        eulerzyx = FRSXform * eulerzyx;

                        cPose peulerxyz = eulerzyx.getPoseEulerXYZ();
                        Console.WriteLine($"eZYX: rz={rz,8:F3} ry={ry,8:F3} rx={rx,8:F3} eXYZ rx={peulerxyz.rx,8:F3}, ry={peulerxyz.ry,8:F3}, rz={peulerxyz.rz,8:F3}");

                    }
                }

            }

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
