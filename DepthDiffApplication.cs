using CommandLine;
using JointMVS;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using OpenCvSharp.CPlusPlus;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;

namespace DepthDiff
{
    public class DepthDiffApplication : ConsoleApplication<DepthDiffOption>
    {
        OctreeNode sourceNode;
        Mesh sourceMesh;
        List<OctreeNode> targetNodes;
        List<Mesh> targetMeshes;
        List<double> rmsePerTargets;
        List<double> angleErrorPerTargets;
        List<List<double>> rmsePerCameras;
        List<List<double>> angleErrorPerCameras;
        List<string> inputName;
        List<Camera> cameras;

        public DepthDiffApplication()
        {
            this.Title = "Depth Diff";
            this.Author = "Kichang Kim";

            Log.RegisterWriter(new ConsoleLogWriter());
        }

        protected override int Run(DepthDiffOption option)
        {
            Log.WriteLine("Start Depth Diff");
            Log.WriteLine();

            if (!ExistCheck(option))
            {
                return -1;
            }

            if (!LoadData(option))
            {
                return -1;
            }

            CalculateError(option);

            SaveResult(option);

            return 0;
        }

        bool ExistCheck(DepthDiffOption option)
        {
            if (!Directory.Exists(option.OutputPath))
            {
                Directory.CreateDirectory(option.OutputPath);
            }

            if (!File.Exists(option.CameraPath))
            {
                Console.WriteLine("Camera file does not exists : {0}", option.CameraPath);
                return false;
            }

            if (!File.Exists(option.SourceMeshPath))
            {
                Console.WriteLine("Source mesh file does not exists : {0}", option.SourceMeshPath);
                return false;
            }
            //inputName = new List<string>();
            //for (int i = 36; i < 39; i =i+1) {
            //    string sTmp = string.Format("C:/Users/Titan/Documents/Visual Studio 2015/Projects/JointMVS/inputs/joyfull/JMVS-Output/{0}.ply", i);
            //    inputName.Add(sTmp);
            //}
            //option.TargetPathList = inputName;
            for (int i = 0; i < option.TargetPathList.Count(); i++)
            {
                string targetPath = option.TargetPathList.ElementAt(i);

                if (!File.Exists(targetPath))
                {
                    Console.WriteLine("Target mesh file does not exists : {0}", targetPath);
                    return false;
                }
            }

            return true;
        }

        bool LoadData(DepthDiffOption option)
        {
            this.StartSection("Loading");

            this.targetNodes = new List<OctreeNode>();
            this.targetMeshes = new List<Mesh>();

            try
            {
                this.StartStep(string.Format("Loading cameras : {0} ...", option.CameraPath));
                this.cameras = Camera.LoadFromCameraV2(option.CameraPath);
                this.EndStep();

                this.StartStep(string.Format("Loading source mesh (flip {0}) : {1} ... ", !option.NoFlip, option.SourceMeshPath));
                this.sourceMesh = Mesh.LoadFromPly(option.SourceMeshPath, !option.NoFlip);
                this.EndStep();

                this.StartStep(string.Format("Calculating normals ... ", !option.NoFlip, option.SourceMeshPath));
                this.sourceMesh.ReCalculateNormal();
                this.EndStep();

                this.StartStep("Generating source octree ...");
                this.sourceNode = this.GetNodeFromMesh(sourceMesh);
                this.EndStep();

                foreach (string targetPath in option.TargetPathList)
                {
                    this.StartStep(string.Format("Loading target mesh ({0}) ...", targetPath));
                    Mesh targetMesh = Mesh.LoadFromPly(targetPath);
                    this.targetMeshes.Add(targetMesh);
                    this.EndStep();

                    this.StartStep(string.Format("Calculating normals ... ", !option.NoFlip, option.SourceMeshPath));
                    targetMesh.ReCalculateNormal();
                    this.EndStep();

                    this.StartStep("Generating target octree ...");
                    this.targetNodes.Add(this.GetNodeFromMesh(targetMesh));
                    this.EndStep();
                }

            }
            catch (Exception e)
            {
                Console.WriteLine(e);
                return false;
            }

            this.EndSection();

            return true;
        }

        void CalculateError(DepthDiffOption option)
        {
            this.StartSection("Error Calculation");

            this.rmsePerTargets = new List<double>();
            this.rmsePerCameras = new List<List<double>>();
            this.angleErrorPerTargets = new List<double>();
            this.angleErrorPerCameras = new List<List<double>>();

            this.StartStep("Generating source depth maps and normal maps ...");
            List<double[]> sourceDepthMaps = new List<double[]>();
            List<Vector3[]> sourceNormals = new List<Vector3[]>();
            
            foreach (Camera camera in this.cameras)
            {
                double[] depthMap;
                Vector3[] normals;
                Mat depthMat, normalMat;

                this.GenerateDepthAndNormalMap(camera, sourceNode, sourceMesh, out depthMap, out depthMat, out normalMat, out normals);

                string imageFilename = Path.GetFileNameWithoutExtension(camera.FileName);

                depthMat.SaveImage(Path.Combine(option.OutputPath, string.Format("{0}_depth_source.png", imageFilename)));
                normalMat.SaveImage(Path.Combine(option.OutputPath, string.Format("{0}_normal_source.png", imageFilename)));

                depthMat.Dispose();
                normalMat.Dispose();

                sourceDepthMaps.Add(depthMap);
                sourceNormals.Add(normals);
            }
            this.EndStep();

            List<List<double[]>> targetDepthMaps = new List<List<double[]>>();
            List<List<Vector3[]>> targetNormals = new List<List<Vector3[]>>();

            this.StartStep("Generating target depth maps and normal maps ...");
            for (int i = 0; i < option.TargetPathList.Count(); i++)
            {
                List<double[]> depthmaps = new List<double[]>();
                List<Vector3[]> normalsList = new List<Vector3[]>();

                for (int ci = 0; ci < this.cameras.Count; ci++)
                {
                    Camera camera = this.cameras[ci];

                    double[] depthMap;
                    Mat depthMat, normalMat;

                    Vector3[] normals;
                    this.GenerateDepthAndNormalMap(camera, this.targetNodes[i], this.targetMeshes[i], out depthMap, out depthMat, out normalMat, out normals);

                    depthmaps.Add(depthMap);
                    normalsList.Add(normals);

                    string imageFilename = Path.GetFileNameWithoutExtension(camera.FileName);

                    depthMat.SaveImage(Path.Combine(option.OutputPath, string.Format("{0}_depth_target_{1}.png", imageFilename, i)));
                    normalMat.SaveImage(Path.Combine(option.OutputPath, string.Format("{0}_normal_target_{1}.png", imageFilename, i)));

                    depthMat.Dispose();
                    normalMat.Dispose();
                }

                targetDepthMaps.Add(depthmaps);
                targetNormals.Add(normalsList);
            }
            this.EndStep();

            this.StartStep("Calculating errors ...");
            List<List<double>> OverMaxError = new List<List<double>>();
            for (int ci = 0; ci < this.cameras.Count; ci++)
            {
                OverMaxError.Add(new List<double>());
            }
            int[][] OverMaxError_Count = new int[option.TargetPathList.Count()][];
            
            
            for (int i = 0; i < option.TargetPathList.Count(); i++)
            {
                OverMaxError_Count[i] = new int[this.cameras.Count];
                
                for (int ci = 0; ci < this.cameras.Count; ci++)
                {
                    Camera camera = this.cameras[ci];
                    Mat errorMat;

                    string imageFilename = Path.GetFileNameWithoutExtension(camera.FileName);
                    double errorSquareSum;
                    int errorCount;
                    double angleErrorSum;
                    int angleErrorCount;
                    List<double> OverMaxError_perCamera;
                    int OverMaxError_Count_perCamera;


                    this.GenerateMedian(camera, sourceDepthMaps[ci], targetDepthMaps[i][ci], targetDepthMaps.Select(dm => dm[ci]).ToList(), sourceNormals[ci], targetNormals[i][ci], option.ErrorScale, out OverMaxError_perCamera, out OverMaxError_Count_perCamera);
                    
                    OverMaxError[ci].AddRange(OverMaxError_perCamera);
                    OverMaxError_Count[i][ci] += OverMaxError_Count_perCamera;
                }
            }
            double[] medianThreshold = new double[this.cameras.Count];
            for (int j = 0; j < option.TargetPathList.Count(); j++)
            {
                for (int ci = 0; ci < this.cameras.Count; ci++)
                {
                    List<double> List_Tmp = OverMaxError[ci].OrderBy(v => v).ToList();
                    medianThreshold[ci] = List_Tmp.ElementAt(List_Tmp.Count / 2);
                }
            }
            for (int i = 0; i < option.TargetPathList.Count(); i++)
            {
                double totalErrorSquareSum = 0.0;
                int totalErrorCount = 0;
                double totalAngleErrorSum = 0.0;
                int totalAngleErrorCount = 0;

                List<double> rmsePerCamera = new List<double>();
                List<double> angleErrorPerCamera = new List<double>();

                OverMaxError_Count[i] = new int[this.cameras.Count];
                
                for (int ci = 0; ci < this.cameras.Count; ci++)
                {
                    Camera camera = this.cameras[ci];
                    Mat errorMat;

                    string imageFilename = Path.GetFileNameWithoutExtension(camera.FileName);
                    double errorSquareSum;
                    int errorCount;
                    double angleErrorSum;
                    int angleErrorCount;


                    this.GenerateErrorMap(camera, sourceDepthMaps[ci], targetDepthMaps[i][ci], targetDepthMaps.Select(dm => dm[ci]).ToList(), sourceNormals[ci], targetNormals[i][ci], option.ErrorScale, medianThreshold[ci], out errorSquareSum, out errorCount, out errorMat, out angleErrorSum, out angleErrorCount);
                    errorMat.SaveImage(Path.Combine(option.OutputPath, string.Format("{0}_error_target_{1}.png", imageFilename, i)));
                    errorMat.Dispose();

                    double rmse = Math.Sqrt(errorSquareSum / (double)errorCount);
                    rmsePerCamera.Add(rmse);

                    double angleError = angleErrorSum / (double)angleErrorCount;
                    angleErrorPerCamera.Add(angleError);

                    totalErrorSquareSum += errorSquareSum;
                    totalErrorCount += errorCount;

                    totalAngleErrorSum += angleErrorSum;
                    totalAngleErrorCount += angleErrorCount;

                }

                this.rmsePerCameras.Add(rmsePerCamera);
                this.angleErrorPerCameras.Add(angleErrorPerCamera);

                double rmsePerTarget = Math.Sqrt(totalErrorSquareSum / (double)totalErrorCount);
                this.rmsePerTargets.Add(rmsePerTarget);

                double angleErrorPerTarget = totalAngleErrorSum / (double)totalAngleErrorCount;
                this.angleErrorPerTargets.Add(angleErrorPerTarget);
            }
            
            this.EndStep();


            this.EndSection();
        }

        void GenerateDepthAndNormalMap(Camera camera, OctreeNode node, Mesh mesh, out double[] outputDepths, out Mat outputDepthMat, out Mat outputNormalMat, out Vector3[] outputNormals)
        {
            double[] depths = new double[camera.Width * camera.Height];
            outputDepthMat = Mat.Zeros(new Size(camera.Width, camera.Height), MatType.CV_8UC3);
            Mat normalMat = Mat.Zeros(new Size(camera.Width, camera.Height), MatType.CV_8UC3);
            Vector3[] normals = new Vector3[camera.Width * camera.Height];

            Parallel.For(0, camera.Height, y =>
            {
                for (int x = 0; x < camera.Width; x++)
                {
                    int offset = y * camera.Width + x;
                    depths[offset] = -1.0;
                    normals[offset] = Vector3::Zero();
                    Vector3 localTarget = new Vector3(((double)x - camera.CX) / camera.FX, ((double)y - camera.CY) / camera.FY, 1.0);
                    Vector3 worldTarget = Vector3.Transform(camera.World, localTarget);
                    Vector3 worldOrigin = Vector3.Transform(camera.World, Vector3.Zero);
                    Vector3 direction = (worldTarget - worldOrigin).Normalized;
                    Vector3 localDirection = localTarget.Normalized;

                    List<double> distances = new List<double>();
                    List<int> resultFaces = new List<int>();

                    node.TestRay(mesh, ref worldOrigin, ref direction, resultFaces, distances, false);

                    if (distances.Count != 0)
                    {
                        double minDistance = double.MaxValue;
                        Vector3 normal = Vector3.Zero;

                        for (int fi = 0; fi < distances.Count; fi++)
                        {
                            if (minDistance > distances[fi])
                            {
                                minDistance = distances[fi];

                                Face face = mesh.Faces[resultFaces[fi]];

                                normal = mesh.Normals[face.V0] + mesh.Normals[face.V1] + mesh.Normals[face.V2];

                                if (!double.IsNaN(normal.X) && normal.Length > 0.5)
                                {
                                    normal = normal.Normalized;
                                }
                                else
                                {
                                    normal = Vector3.Zero;
                                }
                            }
                        }

                        depths[offset] = (localDirection * minDistance).Z;
                        
                        if (normal.Length > 0.5)
                        {
                            normals[offset] = normal;

                            Matrix4 cameraRotation = Matrix4.FromQuaternion(Quaternion.Invert(camera.Rotation));
                            normal = Vector3.Transform(cameraRotation, normal).Normalized;

                            ImageHelper.SetPixel(normalMat, y, x,
                                    new Vector3((normal.X + 1.0) * 0.5 * 255.0, (-normal.Y + 1.0) * 0.5 * 255.0, (-normal.Z + 1.0) * 0.5 * 255.0));
                        }
                    }
                }
            });

            double maxDepth = double.MinValue;
            double minDepth = double.MaxValue;

            for (int y = 0; y < camera.Height; y++)
            {
                for (int x = 0; x < camera.Width; x++)
                {
                    int offset = y * camera.Width + x;

                    double depth = depths[offset];

                    if (depth > 0)
                    {
                        if (depth < minDepth)
                        {
                            minDepth = depth;
                        }

                        if (depth > maxDepth)
                        {
                            maxDepth = depth;
                        }
                    }
                }
            }

            if (maxDepth - minDepth > 0)
            {
                for (int y = 0; y < camera.Height; y++)
                {
                    for (int x = 0; x < camera.Width; x++)
                    {
                        int offset = y * camera.Width + x;

                        double depth = depths[offset];

                        if (depth > 0)
                        {
                            double scaledDepth = (depth - minDepth) / (maxDepth - minDepth);

                            double r, g, b;

                            ImageHelper.PseudoColor(scaledDepth, out r, out g, out b);
                            ImageHelper.SetPixel(outputDepthMat, y, x, new Vector3(Math.Min(r * 255.0, 255.0), Math.Min(g * 255.0, 255.0), Math.Min(b * 255.0, 255.0)));
                        }
                    }
                }
            }

            outputNormalMat = normalMat;
            outputDepths = depths;
            outputNormals = normals;
        }

        void GenerateErrorMap(Camera camera, double[] sourceDepthMap, double[] targetDepthMap, List<double[]> otherDepthMaps, Vector3[] sourceNormals, Vector3[] targetNormals, double scale, double medianThreshold, out double errorSquareSum, out int errorCount, out Mat errorMat, out double angleErrorSum, out int angleErrorCount)
        {
            errorMat = Mat.Zeros(new Size(camera.Width, camera.Height), MatType.CV_8UC3);

            errorSquareSum = 0.0;
            errorCount = 0;
            angleErrorSum = 0.0;
            angleErrorCount = 0;
            
            for (int y = 0; y < camera.Height; y++)
            {
                for (int x = 0; x < camera.Width; x++)
                {
                    int offset = y * camera.Width + x;
                    bool isCandidate = true;

                    double sourceDepth = sourceDepthMap[offset];
                    Vector3 sourceNormal = sourceNormals[offset];
                    double targetDepth = targetDepthMap[offset];
                    Vector3 targetNormal = targetNormals[offset];

                    if (sourceDepth <= 0 || targetDepth <= 0)
                    {
                        continue;
                    }

                    foreach (double[] otherDepthMap in otherDepthMaps)
                    {
                        double otherDepth = otherDepthMap[offset];

                        if (otherDepth <= 0 || Math.Abs(otherDepth - sourceDepth) >= medianThreshold)
                        {
                            isCandidate = false;
                            break;
                        }
                    }

                    if (isCandidate)
                    {
                        double error = sourceDepth - targetDepth;
                        if (Math.Abs(error) < medianThreshold)
                        {
                            errorSquareSum += error * error;
                            errorCount += 1;
                            double r, g, b;
                            ImageHelper.PseudoColor(Math.Min(1.0, Math.Abs(error / scale)), out r, out g, out b);
                            ImageHelper.SetPixel(errorMat, y, x, new Vector3(r, g, b) * 255.0);
                        }
                        else
                        {
                            ImageHelper.SetPixel(errorMat, y, x, new Vector3(0, 0, 0));
                        }
                        
                        double angleError = Math.Acos(Vector3.Dot(sourceNormal, targetNormal)) * 180.0 / Math.PI;
                        angleErrorSum += angleError;
                        angleErrorCount += 1;
                    }
                }
            }
        }

        void GenerateErrorMapSimple(Camera camera, double[] sourceDepthMap, double[] targetDepthMap, 
                                    Vector3[] sourceNormals, Vector3[] targetNormals,
                                    out double errorSquareSum, out int errorCount, out Mat errorMat, out double angleErrorSum, out int angleErrorCount)
        {
             errorMat = Mat.Zeros(new Size(camera.Width, camera.Height), MatType.CV_8UC3);

            errorSquareSum = 0.0;
            errorCount = 0;
            angleErrorSum = 0.0;
            angleErrorCount = 0;
            
            for (int y = 0; y < camera.Height; y++)
            {
                for (int x = 0; x < camera.Width; x++)
                {
                    int offset = y * camera.Width + x;

                    double sourceDepth = sourceDepthMap[offset];
                    Vector3 sourceNormal = sourceNormals[offset];
                    double targetDepth = targetDepthMap[offset];
                    Vector3 targetNormal = targetNormals[offset];

                    if (sourceDepth <= 0 || targetDepth <= 0)
                    {
                        continue;
                    }

                    double error = sourceDepth - targetDepth;
                    errorSquareSum += error * error;
                    errorCount += 1;
                    double r, g, b;
                    ImageHelper.PseudoColor(Math.Min(1.0, Math.Abs(error / scale)), out r, out g, out b);
                    ImageHelper.SetPixel(errorMat, y, x, new Vector3(r, g, b) * 255.0);
                    
                    double angleError = Math.Acos(Vector3.Dot(sourceNormal, targetNormal)) * 180.0 / Math.PI;
                    angleErrorSum += angleError;
                    angleErrorCount += 1;
                }
            }
        }


        void GenerateMedian(Camera camera, double[] sourceDepthMap, double[] targetDepthMap, List<double[]> otherDepthMaps, Vector3[] sourceNormals, Vector3[] targetNormals, double scale, out List<double> OverMaxError, out int OverMaxErrorCount)
        {
            OverMaxError = new List<double>();
            OverMaxErrorCount = 0;

            for (int y = 0; y < camera.Height; y++)
            {
                for (int x = 0; x < camera.Width; x++)
                {
                    int offset = y * camera.Width + x;
                    bool isCandidate = true;

                    double sourceDepth = sourceDepthMap[offset];
                    Vector3 sourceNormal = sourceNormals[offset];
                    double targetDepth = targetDepthMap[offset];
                    Vector3 targetNormal = targetNormals[offset];

                    if (sourceDepth <= 0 || targetDepth <= 0)
                    {
                        continue;
                    }

                    foreach (double[] otherDepthMap in otherDepthMaps)
                    {
                        double otherDepth = otherDepthMap[offset];

                        if (otherDepth <= 0)
                        {
                            isCandidate = false;
                            break;
                        }
                    }

                    if (isCandidate)
                    {
                        double error = sourceDepth - targetDepth;
                        if (error >= scale)
                        { 
                            OverMaxError.Add(Math.Abs(error));
                            OverMaxErrorCount++;
                        }
                        
                    }
                }
            }
        }
        void SaveResult(DepthDiffOption option)
        {
            this.StartSection("Saving Results");

            this.StartStep("Writing to text file ...");
            using (FileStream stream = new FileStream(Path.Combine(option.OutputPath, "rmse.txt"), FileMode.Create))
            using (StreamWriter writer = new StreamWriter(stream))
            using (JsonTextWriter jsonWriter = new JsonTextWriter(writer))
            {
                jsonWriter.Formatting = Formatting.Indented;

                JObject resultJObject = new JObject();

                resultJObject.Add("error_scale", option.ErrorScale);
                resultJObject.Add("input_source", option.SourceMeshPath);

                JArray targetJArray = new JArray();
                foreach (string targetPath in option.TargetPathList)
                {
                    targetJArray.Add(targetPath);
                }

                resultJObject.Add("input_targets", targetJArray);

                JArray totalRmseJArray = new JArray();
                foreach (double rmse in this.rmsePerTargets)
                {
                    totalRmseJArray.Add(rmse);
                }

                resultJObject.Add("total_rmse", totalRmseJArray);

                JArray totalAngleErrorJArray = new JArray();
                foreach (double angleError in this.angleErrorPerTargets)
                {
                    totalAngleErrorJArray.Add(angleError);
                }

                resultJObject.Add("total_angle_error", totalAngleErrorJArray);

                JArray rmsePerCameraJArray = new JArray();
                for (int ci = 0; ci < this.cameras.Count; ci++)
                {
                    Camera camera = this.cameras[ci];

                    JObject rmsePerCameraJObject = new JObject();
                    rmsePerCameraJObject.Add("image_filename", camera.FileName);

                    JArray rmseJArray = new JArray();
                    for (int ti = 0; ti < this.rmsePerCameras.Count; ti++)
                    {
                        rmseJArray.Add(this.rmsePerCameras[ti][ci]);
                    }
                    rmsePerCameraJObject.Add("rmse", rmseJArray);

                    rmsePerCameraJArray.Add(rmsePerCameraJObject);
                }
                resultJObject.Add("rmse_per_camera", rmsePerCameraJArray);

                JArray angleErrorPerCameraJArray = new JArray();
                for (int ci = 0; ci < this.cameras.Count; ci++)
                {
                    Camera camera = this.cameras[ci];

                    JObject angleErrorPerCameraJObject = new JObject();
                    angleErrorPerCameraJObject.Add("image_filename", camera.FileName);

                    JArray angleErrorJArray = new JArray();
                    for (int ti = 0; ti < this.angleErrorPerCameras.Count; ti++)
                    {
                        angleErrorJArray.Add(this.angleErrorPerCameras[ti][ci]);
                    }
                    angleErrorPerCameraJObject.Add("angle_error", angleErrorJArray);

                    angleErrorPerCameraJArray.Add(angleErrorPerCameraJObject);
                }
                resultJObject.Add("angle_error_per_camera", angleErrorPerCameraJArray);

                resultJObject.WriteTo(jsonWriter);
            }

            this.EndStep();
            this.EndSection();
        }

        OctreeNode GetNodeFromMesh(Mesh mesh)
        {
            OctreeNode node = new OctreeNode();
            BoundingBox bb = BoundingBox.Create(mesh.Vertices);
            node.Box = bb;

            int octreeLevel = 0;

            int gap = 1;

            while (gap < mesh.Vertices.Count)
            {
                gap *= 8;
                octreeLevel++;
            }

            octreeLevel += 3;

            Parallel.For(0, mesh.Faces.Count, (face) =>
            {
                node.Add(mesh, face, octreeLevel);
            }
            );

            return node;
        }
    }

    public class DepthDiffOption
    {
        [Option('s', "source", Required = true,
          HelpText = "Source mesh path.")]
        public string SourceMeshPath { get; set; }

        [Option('c', "camera", Required = true,
          HelpText = "CameraV2 file path.")]
        public string CameraPath { get; set; }

        [Option("noflip", Required = false,
          HelpText = "Switch. Disable face flip for source mesh.")]
        public bool NoFlip { get; set; }

        [Option('o', "output", Required = false, Default = "Error",
          HelpText = "Output folder path.")]
        public string OutputPath { get; set; }

        [Option('e', "error", Required = false, Default = 0.05,
          HelpText = "Error scale for visualizaion.")]
        public double ErrorScale
        {
            get; set;
        }

        [Value(0, HelpText = "Target mesh file paths.", Required = true)]
        public IEnumerable<string> TargetPathList
        {
            get; set;
        }
    }
}
