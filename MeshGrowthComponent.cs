using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace MeshGrowth
{
    public class MeshGrowthComponent : GH_Component
    {
        private MeshGrowthSystem _meshGrowthSystem;

        public MeshGrowthComponent()
          : base("MeshGrowth", "MeshGrowth",
              "Description",
              "Mesh", "Custom")
        {
        }
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Reset", "Reset", "Reset", GH_ParamAccess.item);
            pManager.AddMeshParameter("Starting Mesh", "StartingMesh", "StartingMesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Subiteration Count", "Subiteration Count", "Subiteration Count", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Grow", "Grow", "Grow", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Max. Vertex Count", "Max. Vertex Count", "Max. Vertex Count", GH_ParamAccess.item);
            pManager.AddNumberParameter("Edge Length Constraint Weight", "Edge Length Constraint Weight", "Edge Length Constraint Weight", GH_ParamAccess.item);
            pManager.AddNumberParameter("Collision Distance", "Collision Distance", "Collision Distance", GH_ParamAccess.item);
            pManager.AddNumberParameter("Collision Weight", "Collision Weight", "Collision Weight", GH_ParamAccess.item);
            pManager.AddNumberParameter("Bending Resistance Weight", "Bending Resistance Weight", "Bending Resistance Weight", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Use R-Tree", "Use R-Tree", "Use R-Tree", GH_ParamAccess.item);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "Mesh", "Mesh", GH_ParamAccess.item);
        }


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool iReset = true;
            Mesh iStartingMesh = null;
            int iSubiterationCount = 0;
            bool iGrow = false;
            int iMaxVertexCount = 0;
            double iEdgeLengthConstrainWeight = 0.0;
            double iCollisionDistance = 0.0;
            double iCollisionWeight = 0.0;
            double iBendingResistanceWeight = 0.0;
            bool iUseRTree = false;

            DA.GetData("Reset", ref iReset);
            DA.GetData("Starting Mesh", ref iStartingMesh);
            DA.GetData("Subiteration Count", ref iSubiterationCount);
            DA.GetData("Grow", ref iGrow);
            DA.GetData("Max. Vertex Count", ref iMaxVertexCount);
            DA.GetData("Edge Length Constraint Weight", ref iEdgeLengthConstrainWeight);
            DA.GetData("Collision Distance", ref iCollisionDistance);
            DA.GetData("Collision Weight", ref iCollisionWeight);
            DA.GetData("Bending Resistance Weight", ref iBendingResistanceWeight);
            DA.GetData("Use R-Tree", ref iUseRTree);

            if (iReset || _meshGrowthSystem == null)
                _meshGrowthSystem = new MeshGrowthSystem(iStartingMesh);

            _meshGrowthSystem.Grow = iGrow;
            _meshGrowthSystem.MaxVertexCount = iMaxVertexCount;
            _meshGrowthSystem.EdgeLengthConstraintWeight = iEdgeLengthConstrainWeight;
            _meshGrowthSystem.CollisionWeight = iCollisionWeight;
            _meshGrowthSystem.BendingResistanceWeight = iBendingResistanceWeight;
            _meshGrowthSystem.UseRTree = iUseRTree;
            _meshGrowthSystem.CollisionDistance = iCollisionDistance;

            _meshGrowthSystem.Update();

            DA.SetData("Mesh", _meshGrowthSystem.GetRhinoMesh());
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
            get { return new Guid("b0d789f2-d21c-4e7e-b5f2-3d1664e4e26f"); }
        }
    }
}
