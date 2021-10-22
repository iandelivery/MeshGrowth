using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace MeshGrowth
{
    public struct PathPoint
    {
        public PathPoint(PathPoint other)
        {
            PreviousId = other.PreviousId;
            TotalDistance = other.TotalDistance;
            X = other.X;
            Y = other.Y;
        }

        public PathPoint(double x, double y, double totalDistance, int previousId)
        {
            PreviousId = previousId;
            TotalDistance = totalDistance;
            X = x;
            Y = y;
        }
        public PathPoint(Point3d point, double totalDistance, int previousId)
        {
            PreviousId = previousId;
            TotalDistance = totalDistance;
            X = point.X;
            Y = point.Y;
        }
        public int PreviousId { get; set; }
        public double TotalDistance { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
    }
    /// <summary>
    /// http://alienryderflex.com/shortest_path/
    /// </summary>
    public class ShortestPath
    {
        private bool PointInPolygonSet(double testX, double testY, List<Polyline> allPolys)
        {
            bool oddNodes = false;

            for (int id = 0; id < allPolys.Count; id++)
            {
                Polyline pl = allPolys[id];
                for (int i = 0; i < pl.SegmentCount; i++)
                {
                    int j = i + 1; if (j == pl.SegmentCount) j = 0;
                    if (pl.Y[i] < testY
                        && pl.Y[j] >= testY
                        || pl.Y[j] < testY
                    && pl.Y[i] >= testY)
                    {
                        if (pl.X[i] + (testY - pl.Y[i])
                            / (pl.Y[j] - pl.Y[i])
                            * (pl.X[j] - pl.X[i]) < testX)
                        {
                            oddNodes = !oddNodes;
                        }
                    }
                }
            }
            return oddNodes;
        }
        //  This function should be called with the full set of *all* relevant polygons.
        //  (The algorithm automatically knows that enclosed polygons are “no-go”
        //  areas.)
        //
        //  Note:  As much as possible, this algorithm tries to return true when the
        //         test line-segment is exactly on the border of the polygon, particularly
        //         if the test line-segment *is* a side of a polygon.

        private bool LineInPolygonSet(double testSX, double testSY, double testEX, double testEY, List<Polyline> allPolys)
        {

            double theCos, theSin, dist, sX, sY, eX, eY, rotSX, rotSY, rotEX, rotEY, crossX;

            testEX -= testSX;
            testEY -= testSY; dist = Math.Sqrt(testEX * testEX + testEY * testEY);
            theCos = testEX / dist;
            theSin = testEY / dist;

            for (int polyI = 0; polyI < allPolys.Count; polyI++)
            {
                Polyline pl = allPolys[polyI];
                for (int i = 0; i < pl.SegmentCount; i++)
                {
                    int j = i + 1; if (j == pl.SegmentCount) j = 0;

                    sX = pl.X[i] - testSX;
                    sY = pl.Y[i] - testSY;
                    eX = pl.X[j] - testSX;
                    eY = pl.Y[j] - testSY;
                    if (sX == 0.0 && sY == 0.0 && eX == testEX && eY == testEY
                        || eX == 0.0 && eY == 0.0 && sX == testEX && sY == testEY)
                    {
                        return true;
                    }

                    rotSX = sX * theCos + sY * theSin;
                    rotSY = sY * theCos - sX * theSin;
                    rotEX = eX * theCos + eY * theSin;
                    rotEY = eY * theCos - eX * theSin;
                    if (rotSY < 0.0 && rotEY > 0.0
                    || rotEY < 0.0 && rotSY > 0.0)
                    {
                        crossX = rotSX + (rotEX - rotSX) * (0.0 - rotSY) / (rotEY - rotSY);
                        if (crossX >= 0.0 && crossX <= dist) return false;
                    }

                    if (rotSY == 0.0 && rotEY == 0.0
                        && (rotSX >= 0.0 || rotEX >= 0.0)
                        && (rotSX <= dist || rotEX <= dist)
                        && (rotSX < 0.0 || rotEX < 0.0
                    || rotSX > dist || rotEX > dist))
                    {
                        return false;
                    }
                }
            }

            return PointInPolygonSet(testSX + testEX / 2.0, testSY + testEY / 2.0, allPolys);
        }
        double CalcDist(double sX, double sY, double eX, double eY)
        {
            eX -= sX; eY -= sY;
            return Math.Sqrt(eX * eX + eY * eY);
        }
        private unsafe void SwapPoints(PathPoint a, PathPoint b)
        {
            PathPoint swap = new PathPoint(a);
            a = b;
            b = swap;
        }
        //  Finds the shortest path from sX,sY to eX,eY that stays within the polygon set.
        //
        //  Note:  To be safe, the solutionX and solutionY arrays should be large enough
        //         to accommodate all the corners of your polygon set (although it is
        //         unlikely that anywhere near that many elements will ever be needed).
        //
        //  Returns true if the optimal solution was found, or false if there is no solution.
        //  If a solution was found, solutionX and solutionY will contain the coordinates
        //  of the intermediate nodes of the path, in order.  (The startpoint and endpoint
        //  are assumed, and will not be included in the solution.)

        private unsafe bool RunShortestPath(double sX, double sY, double eX, double eY, List<Polyline> allPolys,
        double* solutionX, double* solutionY, int* solutionNodes)
        {

            double INF = 9999999;    //  (larger than total solution dist could ever be)

            //point pointList[1000] ;   //  (enough for all polycorners plus two)
            //int pointCount;
            List<PathPoint> pathPointList = new List<PathPoint>();
            int pointCount;
            int treeCount, polyI, i, j, bestI, bestJ;
            double bestDist, newDist;

            //  Fail if either the startpoint or endpoint is outside the polygon set.
            if (!PointInPolygonSet(sX, sY, allPolys)
            || !PointInPolygonSet(eX, eY, allPolys))
            {
                return false;
            }

            //  If there is a straight-line solution, return with it immediately.
            if (LineInPolygonSet(sX, sY, eX, eY, allPolys))
            {
                //solutionNodes = null;
                return true;
            }

            //  Build a point list that refers to the corners of the
            //  polygons, as well as to the startpoint and endpoint.
            pathPointList.Add(new PathPoint(sX, sY, 0.0, -1));
            pathPointList.Add(new PathPoint());
            pointCount = 1;
            for (polyI = 0; polyI < allPolys.Count; polyI++)
            {
                Polyline pl = allPolys[polyI];
                for (i = 0; i < pl.SegmentCount; i++)
                {
                    var pathPoint = new PathPoint(pl.X[i], pl.Y[i], 0.0, -1);
                    pathPointList[pointCount] = pathPoint;
                    pathPointList.Add(new PathPoint());
                    pointCount++;
                }
            }
            var endPoint = pathPointList[pointCount];
            endPoint.X = eX;
            endPoint.Y = eY;
            pathPointList.Add(new PathPoint());
            pointCount++;

            //  Initialize the shortest-path tree to include just the startpoint.
            treeCount = 1;
            //pathPointList[0].TotalDistance = 0.0;
            //  Iteratively grow the shortest-path tree until it reaches the endpoint
            //  -- or until it becomes unable to grow, in which case exit with failure.
            bestI = -1;
            bestJ = 0;
            while (bestJ < pointCount - 1)
            {
                bestDist = INF;
                for (i = 0; i < treeCount; i++)
                {
                    for (j = treeCount; j < pointCount; j++)
                    {
                        if (LineInPolygonSet(
                            pathPointList[i].X, pathPointList[i].Y,
                            pathPointList[j].X, pathPointList[j].Y, allPolys))
                        {
                            newDist = pathPointList[i].TotalDistance + CalcDist(
                                pathPointList[i].X, pathPointList[i].Y,
                                pathPointList[j].X, pathPointList[j].Y);
                            if (newDist < bestDist)
                            {
                                bestDist = newDist;
                                bestI = i;
                                bestJ = j;
                            }
                        }
                    }
                }
                if (bestDist == INF)
                    return false;   //  (no solution)
                double x = pathPointList[bestJ].X;
                double y = pathPointList[bestJ].Y;
                pathPointList[bestJ] = new PathPoint(x, y, bestDist, bestI);
                SwapPoints(pathPointList[bestJ], pathPointList[treeCount]);
                treeCount++;
            }

            //  Load the solution arrays.
            *solutionNodes = -1; i = treeCount - 1;
            while (i > 0)
            {
                i = pathPointList[i].PreviousId; 
                (*solutionNodes)++;
            }
            j = (*solutionNodes) - 1;
            i = treeCount - 1;
            while (j >= 0)
            {
                i = pathPointList[i].PreviousId;
                solutionX[j] = pathPointList[i].X;
                solutionY[j] = pathPointList[i].Y;
                j--;
            }

            //  Success.
            return true;
        }
    }
}
