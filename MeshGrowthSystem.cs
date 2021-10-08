using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Plankton;
using PlanktonGh;
using Rhino.Geometry;

namespace MeshGrowth
{
    public class MeshGrowthSystem
    {
        private PlanktonMesh _ptMesh;
        private List<Vector3d> _totalWeightedMoves;
        private List<double> _totalWeights;
        //private int _vertexCount;
        public MeshGrowthSystem(Mesh startingMesh)
        {
            _ptMesh = startingMesh.ToPlanktonMesh();
            //_vertexCount = _ptMesh.Vertices.Count;
        }

        public Mesh GetRhinoMesh()
        {
            return _ptMesh.ToRhinoMesh();
        }

        public void Update()
        { 
            if(Grow)
            {
                SplitAllLongEdges();
            }
            int vertexCount = _ptMesh.Vertices.Count;
            _totalWeightedMoves = new List<Vector3d>(Enumerable.Repeat(new Vector3d(), vertexCount));
            _totalWeights = new List<double>(Enumerable.Repeat(0.0, vertexCount));

            if (UseRTree)
            {
                ProcessCollisionUsingRTree();
            }
            else
            {
                ProcessCollision();
            }
            ProcessBendingResistance();
            UpdateVertexPositions();
            ProcessEdgeLengthConstraint();
        }
        private void ProcessCollisionUsingRTree()
        {
            RTree rTree = new RTree();
            for (int i = 0; i < _ptMesh.Vertices.Count; i++)
            {
                rTree.Insert(_ptMesh.Vertices[i].ToPoint3d(), i);
            }
            for (int i = 0; i < _ptMesh.Vertices.Count; i++)
            {
                Point3d vI = _ptMesh.Vertices[i].ToPoint3d();
                Sphere searchSphere = new Sphere(vI, CollisionDistance);

                List<int> collisionIndices = new List<int>();

                rTree.Search(
                    searchSphere,
                    (sender, args) => { if (i < args.Id) collisionIndices.Add(args.Id); });

                foreach (int j in collisionIndices)
                {
                    Vector3d move = _ptMesh.Vertices[j].ToPoint3d() - _ptMesh.Vertices[i].ToPoint3d();
                    double currentDistance = move.Length;
                    move *= 0.5 * (currentDistance - CollisionDistance) / currentDistance;
                    _totalWeightedMoves[i] += CollisionWeight * move;
                    _totalWeightedMoves[j] -= CollisionWeight * move;
                    _totalWeights[i] += CollisionWeight;
                    _totalWeights[j] += CollisionWeight;
                }
            }
        }

        private void ProcessEdgeLengthConstraint()
        {
            for (int k = 0; k < _ptMesh.Halfedges.Count; k += 2)
            {
                int i = _ptMesh.Halfedges[k].StartVertex;
                int j = _ptMesh.Halfedges[k + 1].StartVertex;

                Point3d ptI = _ptMesh.Vertices[i].ToPoint3d();
                Point3d ptJ = _ptMesh.Vertices[j].ToPoint3d();

                if (ptI.DistanceTo(ptJ) < CollisionDistance) continue;

                Vector3d move = ptJ - ptI;
                move *= (move.Length - CollisionDistance) * 0.5 / move.Length;

                _totalWeightedMoves[i] += move * EdgeLengthConstraintWeight;
                _totalWeightedMoves[j] -= move * EdgeLengthConstraintWeight;
                _totalWeights[i] += EdgeLengthConstraintWeight;
                _totalWeights[j] += EdgeLengthConstraintWeight;
            }
        }
        private void ProcessBendingResistance()
        {
            int halfEdgeCount = _ptMesh.Halfedges.Count;

            for (int k = 0; k < halfEdgeCount; k += 2)
            {
                int i = _ptMesh.Halfedges[k].StartVertex;
                int j = _ptMesh.Halfedges[k + 1].StartVertex;
                int p = _ptMesh.Halfedges[_ptMesh.Halfedges[k].PrevHalfedge].StartVertex;
                int q = _ptMesh.Halfedges[_ptMesh.Halfedges[k + 1].PrevHalfedge].StartVertex;

                Point3d vI = _ptMesh.Vertices[i].ToPoint3d();
                Point3d vJ = _ptMesh.Vertices[j].ToPoint3d();
                Point3d vP = _ptMesh.Vertices[p].ToPoint3d();
                Point3d vQ = _ptMesh.Vertices[q].ToPoint3d();

                Vector3d nP = Vector3d.CrossProduct(vJ - vI, vP - vI);
                Vector3d nQ = Vector3d.CrossProduct(vQ - vI, vJ - vI);

                Vector3d planeNormal = (nP + nQ);
                Point3d planeOrigin = 0.25 * (vI + vJ + vP + vQ);

                Plane plane = new Plane(planeOrigin, planeNormal);

                _totalWeightedMoves[i] += BendingResistanceWeight * (plane.ClosestPoint(vI) - vI);
                _totalWeightedMoves[j] += BendingResistanceWeight * (plane.ClosestPoint(vJ) - vJ);
                _totalWeightedMoves[p] += BendingResistanceWeight * (plane.ClosestPoint(vP) - vP);
                _totalWeightedMoves[q] += BendingResistanceWeight * (plane.ClosestPoint(vQ) - vQ);
                _totalWeights[i] += BendingResistanceWeight;
                _totalWeights[j] += BendingResistanceWeight;
                _totalWeights[p] += BendingResistanceWeight;
                _totalWeights[q] += BendingResistanceWeight;
            }
        }

        private void ProcessCollision()
        {
            int vertexCount = _ptMesh.Vertices.Count;
            for (int i = 0; i < vertexCount; i++)
            {
                for (int j = 0; j < vertexCount; j++)
                {
                    Vector3d move = _ptMesh.Vertices[j].ToPoint3d() -
                        _ptMesh.Vertices[i].ToPoint3d();
                    double currentDistance = move.Length;
                    if (currentDistance > CollisionDistance)
                        continue;
                    move *= 0.5 * (currentDistance - CollisionDistance) / currentDistance;

                    _totalWeightedMoves[i] += move;
                    _totalWeightedMoves[j] -= move;
                    _totalWeights[i] += 1;
                    _totalWeights[j] += 1;
                }
            }
        }
        private void SplitAllLongEdges()
        {
            int halfedgeCount = _ptMesh.Halfedges.Count;
            for (int i = 0; i < halfedgeCount; i+=2)
            {
                if (_ptMesh.Vertices.Count< MaxVertexCount &&
                    _ptMesh.Halfedges.GetLength(i) > 0.99 * CollisionDistance)
                {
                    SplitEdge(i);
                }
            }
        }
        private void SplitEdge(int index)
        {
            int newHalfEdgeIndex = _ptMesh.Halfedges.SplitEdge(index);
            _ptMesh.Vertices.SetVertex(
                _ptMesh.Vertices.Count - 1,
                0.5 * (_ptMesh.Vertices[_ptMesh.Halfedges[index].StartVertex].ToPoint3d() +
                _ptMesh.Vertices[_ptMesh.Halfedges[index + 1].StartVertex].ToPoint3d()));

            if (_ptMesh.Halfedges[index].AdjacentFace >= 0)
                _ptMesh.Faces.SplitFace(newHalfEdgeIndex, _ptMesh.Halfedges[index].PrevHalfedge);

            if (_ptMesh.Halfedges[index + 1].AdjacentFace >= 0)
                _ptMesh.Faces.SplitFace(index + 1, _ptMesh.Halfedges[_ptMesh.Halfedges[index + 1].NextHalfedge].NextHalfedge);
        }
        private void UpdateVertexPositions()
        {
            int vertexCount = _ptMesh.Vertices.Count;
            for (int i = 0; i < vertexCount; i++)
            {
                if (_totalWeights[i] == 0.0)
                    continue;
                Vector3d move = _totalWeightedMoves[i]/ _totalWeights[i];
                Point3d updatedPosistion = _ptMesh.Vertices[i].ToPoint3d() + move;
                _ptMesh.Vertices.SetVertex(i, updatedPosistion.X, updatedPosistion.Y, updatedPosistion.Z);
            }
        }

        public bool Grow { get; set; }
        public int MaxVertexCount { get; set; }
        public double EdgeLengthConstraintWeight { get; set; }
        public double CollisionDistance { get; set; }
        public double CollisionWeight { get; set; }
        public double BendingResistanceWeight { get; set; }
        public bool UseRTree { get; set; }
    }
}
