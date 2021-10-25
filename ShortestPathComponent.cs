using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace MeshGrowth
{
    public class ShortestPathComponent : GH_Component
    {
        private ShortestPath _shortestPath;
        public ShortestPathComponent()
          : base("ShortestPath", "ShortestPath",
              "Description",
              "Mesh", "Custom")
        {
            _shortestPath = new ShortestPath();
        }
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Polylines", "Polylines", "Polylines", GH_ParamAccess.list);
            pManager.AddCurveParameter("Boundary", "Boundary", "Boundary", GH_ParamAccess.item);
            pManager.AddPointParameter("Start", "Start", "Start", GH_ParamAccess.item);
            pManager.AddPointParameter("End", "End", "End", GH_ParamAccess.item);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Path", "Path", "Path", GH_ParamAccess.item);
        }


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> curves = new List<Curve>();
            Curve curve = null;
            Point3d start = Point3d.Origin;
            Point3d end = Point3d.Origin;

            if (!DA.GetDataList("Polylines", curves))
            {
                return;
            }
            if(!DA.GetData("Boundary", ref curve))
            {
                return;
            }
            if (!DA.GetData("Start", ref start)) return;
            if (!DA.GetData("End", ref end)) return;
            List<Polyline> polylines = new List<Polyline>();
            foreach (var crv in curves)
            {
                if (!crv.TryGetPolyline(out Polyline pl))
                    continue;
                polylines.Add(pl);
            }
            if (!curve.TryGetPolyline(out Polyline boundary))
            {
                return;
            }
            //AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Begin algorithm");
            //if(!boundary.Contains(start)) return;
            //if(!boundary.Contains(end)) return;
            polylines.Add(boundary);
            _shortestPath.RunShortestPath(start.X, start.Y, end.X, end.Y, polylines, out List<Point3d> points);

            if(points is null || points.Count==0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "empty points");

            }

            DA.SetData("Path", new Polyline(points));
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("B1EC7CC3-806B-4D2F-B1B5-8E87CE7570E8"); }
        }
    }
}
