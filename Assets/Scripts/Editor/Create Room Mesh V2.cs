using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using DelaunatorSharp;

public class CreateRoomMeshV2
{
    private static AudioReverbZone[] m_ReverbZones;
    private static GameObject m_CurrentCentrePoint;

    [MenuItem("Tools/Geometry/CreateRoomMeshV2")]
    private static void main()
    {
        m_ReverbZones = Object.FindObjectsOfType<AudioReverbZone>();

        foreach (var item in m_ReverbZones)
        {
            getCentrePoint(item);
            List<Vector3> points = getPoints(250); // num points|| max num = 65535 due to int variable type
            // IList<Vector2> points2D = pointsTo2D(points);
            List<Vector3> drape;
            int[] allTriangles = calculatedrapeMesh(points, out drape); //DelaunayTriangulation(points);

            IList<RaycastHit> raycastCollisions = raycasts(points);

            Mesh mesh = new Mesh();
            List<Vector3> CollisionPoints = rcCollisionPoints(raycastCollisions);
            mesh.SetVertices(CollisionPoints); 
            mesh.SetTriangles(allTriangles, 0);
            m_CurrentCentrePoint.GetComponent<MeshFilter>().mesh = mesh;

            List<int> listAllTriangles = new List<int>(allTriangles);
            float SA = geometricCalculations(listAllTriangles, CollisionPoints, raycastCollisions);
        }
    }

    private static void getCentrePoint(AudioReverbZone currentRZ)
    {
        Transform[] childrenTransforms = currentRZ.GetComponentsInChildren<Transform>();

        foreach (var item in childrenTransforms)
        {
            if (item.CompareTag("CentrePoint"))
            {
                //removing componenets
                if (item.gameObject.GetComponent<MeshRenderer>())
                {
                    //GameObject.DestroyImmediate(item.gameObject.GetComponent<MeshRenderer>());
                }
                if (item.gameObject.GetComponent<MeshFilter>())
                {
                    //GameObject.DestroyImmediate(item.gameObject.GetComponent<MeshFilter>());
                }
                m_CurrentCentrePoint = item.gameObject;
            }
        }
    }

    private static List<Vector3> getPoints(int samples)
    {
        List<Vector3> points = new List<Vector3>(samples);
        float phi = Mathf.PI * (3.0f - Mathf.Sqrt(5));  //calculates golden angle in radians
        float fSamples = System.Convert.ToSingle(samples);
        for (int i = 0; i < samples; i++)
        {
            float fI = System.Convert.ToSingle(i);
            float y = 1.0f - (fI / (fSamples - 1.0f)) * 2.0f;  //  y goes from 1 to -1
            float radius = Mathf.Sqrt(1.0f - y * y);   //  radius at y

            float theta = phi * i;  //golden angle increment

            float x = Mathf.Cos(theta) * radius;
            float z = Mathf.Sin(theta) * radius;

            //if (y < 0 && x == z && x == 0)
            //    System.Diagnostics.Debugger.Break();

            Vector3 pnt = new Vector3(x, y, z);

            points.Add(pnt);
        }

        //Debug.Log(points.Count);
        return points;
    }
    private static IList<RaycastHit> raycasts(IList<Vector3> points)
    {
        Vector3 origin = m_CurrentCentrePoint.transform.position;
        IList<RaycastHit> collisions = new List<RaycastHit>(points.Count);

        int i = 0;
        foreach (var direction in points)
        {
            //Debug.Log(direction);
            Ray ray = new Ray(origin, direction);
            Debug.DrawRay(origin, direction, Color.red);
            RaycastHit collision = new RaycastHit();

            if (Physics.Raycast(ray, out collision, Mathf.Infinity))
            {
                //Debug.Log("hit" + collision.point);
                Debug.DrawLine(ray.origin, collision.point, Color.red);
                collisions.Add(collision);
            }
            i++;
        }
        return collisions;
    }

    private static List<int> DelaunayTriangulation(List<Vector3> points)
    {
        IList<Vector2> points2D = pointsTo2D(points);
        List<int> triangles = new List<int>(points.Count);

        Delaunator triangulator = new Delaunator(DelaunatorSharp.Unity.Extensions.DelaunatorExtensions.ToPoints(points2D));
        triangles.AddRange(triangulator.Triangles);

        IPoint[] hullPoints = triangulator.GetHullPoints();
        int hullCount = hullPoints.Length;
        int firstIndex = 0;
        int prevIndex = 0;
        for (int i = 0; i < hullCount; i++)
        {
            Vector2 hullPoint = new Vector2(System.Convert.ToSingle(hullPoints[i].X), System.Convert.ToSingle(hullPoints[i].Y));
            int hullIndex = points2D.IndexOf(hullPoint);
            if (prevIndex != 0)
            {
                if (hullIndex != -1)
                {
                    triangles.Add(prevIndex);
                    triangles.Add(points.Count - 1);
                    triangles.Add(hullIndex);
                }
            }
            if (i == hullCount - 1)
            {
                triangles.Add(hullIndex);
                triangles.Add(points.Count - 1);
                triangles.Add(firstIndex);
            }

            if (firstIndex == 0)
                firstIndex = hullIndex;
            prevIndex = hullIndex;
        }

        return triangles;
    }

    private static int[] calculatedrapeMesh(List<Vector3> points, out List<Vector3> meshPoints)
    {
        meshPoints = new List<Vector3>(points.Count);
        IList<Vector2> points2D = pointsTo2D(points);

        Delaunator triangulator = new Delaunator(DelaunatorSharp.Unity.Extensions.DelaunatorExtensions.ToPoints(points2D));

        for (int i = 0; i < points2D.Count; i++)
        {
            meshPoints.Add(new Vector3(points2D[i].x, points[i].y, points2D[i].y));
        }


        return triangulator.Triangles;
    }

    private static List<Vector2> pointsTo2D(IList<Vector3> points)
    {
        List<Vector2> points2D = new List<Vector2>(points.Count);
        int badPoints = 0;
        foreach (var point in points)
        {
            if (point.x == 0.0f && point.z == 0.0f && point.y < 0.0f)
            {
                badPoints++;
                continue;
            }

            float x = point.x;
            float z = point.z;

            if (point.y < 0.0f)
            {
                float a = 1 / (Mathf.Pow(point.x, 2) + Mathf.Pow(point.z, 2));
                x = a * x;
                z = a * z;
            }

            points2D.Add(new Vector2(x, z));
        }

        return points2D;
    }

    private static List<Vector3> rcCollisionPoints(IList<RaycastHit> raycasts)
    {
        List<Vector3> collisionPoints = new List<Vector3>(raycasts.Count);

        foreach (var item in raycasts)
        {
            collisionPoints.Add(item.point);
        }

        return collisionPoints;
    }

    private static float geometricCalculations(List<int> triangles, List<Vector3> collisionPoints, IList<RaycastHit> raycastCollisions)
    {
        float totalSurfaceArea = 0.0f;
        float totalVolume = 0.0f;
        float[] AvAbsorptionSpec = new float[7];
        float alpha = 0.0f;

        for (int i = 0; i < triangles.Count; i = i + 3)
        {
            float surfaceArea = calcSurfaceArea(i, triangles, collisionPoints);
            //float[] absorptionSpectrum = objectAbsorptionSpectrum(i, triangles, raycastCollisions, surfaceArea);
            //AvAbsorptionSpec  = absorptionSpectrum;
            //alpha = absorptionSpectrum[3] * surfaceArea;
            totalVolume += calcVolume(i, triangles, collisionPoints, surfaceArea);
            totalSurfaceArea += surfaceArea;
        }
        Debug.Log("SF: " + totalSurfaceArea);
        Debug.Log("TV: " + totalVolume);

        return totalSurfaceArea;
    }

    private static float calcSurfaceArea(int i, List<int> triangles, List<Vector3> collisionPoints)
    {
        float side1 = Vector3.Distance(collisionPoints[triangles[i]], collisionPoints[triangles[i + 1]]); // distance round triangle
        float side2 = Vector3.Distance(collisionPoints[triangles[i + 1]], collisionPoints[triangles[i + 2]]);
        float side3 = Vector3.Distance(collisionPoints[triangles[i + 2]], collisionPoints[triangles[i]]);

        float surfaceArea = 0.5f * Mathf.Sqrt((side1 + side2 + side3) * (-side1 + side2 + side3) * (side1 - side2 + side3) * (side1 + side2 - side3)); //heron's formula

        Debug.Log("Surface Area: " + surfaceArea);

        return surfaceArea;
    }
    private static float calcVolume(int i, List<int> triangles, List<Vector3> collisionPoints, float surfaceArea)
    {
        float volume;
        Vector3 origin = m_CurrentCentrePoint.transform.position;

        float length1 = Vector3.Distance(collisionPoints[triangles[i]], origin); // distance round triangle
        float length2 = Vector3.Distance(collisionPoints[triangles[i + 1]], origin);
        float length3 = Vector3.Distance(collisionPoints[triangles[i + 2]], origin);

        float avLength = (length1 + length2 + length3) / 3;
        volume = (surfaceArea * avLength) / 3;

        Debug.Log("Volume: " + volume);

        return volume;
    }
    //outputs average absorption spectrum of all three points
    //private static float[] objectAbsorptionSpectrum(int i, List<int> triangles, IList<RaycastHit> raycastCollisions, float surfaceArea)
    //{
    //    float[] trianglesAbsorption = new float[7];
    //    float ppSurfaceArea = surfaceArea / 3; //surface area divided up pp

    //    for(int point = i; point < (i + 3); i++)
    //    {
    //        GameObject gameobjectAtPoint = raycastCollisions[triangles[i]].collider.gameObject; // gets gameobject
    //        MeshRenderer objectMeshRenderer = gameobjectAtPoint.GetComponent<MeshRenderer>();
    //        Material objectMaterial = objectMeshRenderer.sharedMaterial;
    //        if (objectMaterial != null)
    //        {
    //            trianglesAbsorption = absorptionSpectrumLookup(objectMaterial); //sums absorption spectrums of all three points
    //        }

    //        foreach (int index in trianglesAbsorption)
    //        {
    //            index.Equals(index / 3); // divdes by three to create average spectrum
    //        }
    //    }
    //    //d
    //    foreach (int index in trianglesAbsorption)
    //    {
    //        index.Equals(index / 3);
    //    }

    //    return trianglesAbsorption;
    //}
    ////looks up material absorption spec
    //private static float[] absorptionSpectrumLookup(Material meterial)
    //{
    //    if (meterial.name.Contains("metal"))
    //    {
    //        float[] absorptionSpectrum = new float[7] { 0.01f, 0.01f, 0.01f, 0.02f, 0.02f, 0.02f, 0.02f };
    //    }
    //    else if (meterial.name.Contains("Wood"))
    //    {
    //        float[] absorptionSpectrum = new float[7] { 0.27f, 0.23f, 0.22f, 0.15f, 0.10f, 0.07f, 0.06f };
    //    }
    //    else if (meterial.name.Contains("brick"))
    //    {
    //        float[] absorptionSpectrum = new float[7] { 0.03f, 0.03f, 0.03f, 0.04f, 0.05f, 0.07f, 0.07f };
    //    }
    //    else if (meterial.name.Contains("plaster"))
    //    {
    //        float[] absorptionSpectrum = new float[7] { 0.15f, 0.10f, 0.06f, 0.04f, 0.04f, 0.05f, 0.05f };
    //    }
    //    else if (meterial.name.Contains("tile"))
    //    {
    //        float[] absorptionSpectrum = new float[7] { 0.01f, 0.01f, 0.01f, 0.02f, 0.02f, 0.02f, 0.02f };
    //    }
    //    else if (meterial.name.Contains("absorption"))
    //    {
    //        float[] absorptionSpectrum = new float[7] { 0.22f, 0.60f, 0.92f, 0.90f, 0.88f, 0.88f, 0.88f };
    //    }
    //    else if (meterial.name.Contains("concrete"))
    //    {
    //        float[] absorptionSpectrum = new float[7] { 0.01f, 0.01f, 0.02f, 0.02f, 0.02f, 0.05f, 0.05f };
    //    }
    //    else if (meterial.name.Contains("glass"))
    //    {
    //        float[] absorptionSpectrum = new float[7] { 0.08f, 0.04f, 0.03f, 0.03f, 0.02f, 0.02f, 0.02f };
    //    }
    //    else
    //    {
    //        float[] absorptionSpectrum = new float[7] { 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f };
    //    }

    //    return AbsorptionSpectrum;
    //}
    //private static float[] addFloatArrays(float[] array1, float[] array2)
    //{
    //    float[] outputArray = new float[7];

    //    for (int i = 0; i < 7; i++ )
    //    {
    //        outputArray[i] = array1[i] + array2[i];
    //    }

    //    return outputArray;
    //}
}

