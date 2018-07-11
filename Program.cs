using CommandLine;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DepthDiff
{
    class Program
    {
        public static int Main(string[] args)
        {
            using (DepthDiffApplication app = new DepthDiffApplication())
            {
                int exitCode = app.Start(args);
                return exitCode;
            }
        }
    }
}
