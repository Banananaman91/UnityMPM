﻿using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Jobs;

public class MpmBodyCollision : MonoBehaviour
{
    public struct Particle
    {
        public float2 Position;
        public float2 Velocity;
        public float2x2 AffMomentMatrix;
        public float Mass;
        public float Volume;
        public float2x2 fs; //deformation gradient
    }

    private struct Cell
    {
        public float2 Velocity;
        public float Mass;
    }

    [SerializeField] private int GridRes = 64;
    private int NumCells => GridRes * GridRes;

    // batch size for the job system.
    private const int Division = 16;

    //simulation parameters

    //dt = the time step of the simulation. The stability of the simulation is going to be limited by how much a particle can move in a single time step
    //it's a good rule of thumb to choose dt so that no particle could move more than 1 grid-cell in a single step
    [SerializeField] private float DT = 0.005f;
    [SerializeField] private float DX = 0.33f;
    private int Iterations => (int) (1.0f / DT);
    [SerializeField] private float Gravity = -0.5f;

    // Lamé parameters for stress-strain relationship
    [SerializeField] private float ElasticLambda = 0.0001f;
    [SerializeField] private float ElasticMu = 200;

    public int _numParticles;

    private NativeArray<Particle> _particles;
    private NativeArray<Cell> _grid;
    private NativeArray<Particle> _objectParticle;


    private float2[] _weights = new float2[3];

    private List<float2> _tempPositions;

    [SerializeField] private GameObject _spawnCube;
    [SerializeField] private GameObject _cube;
    private Particle _cubeParticle;
    [SerializeField] private float _cubeMass = 2;
    [SerializeField] private float _cubeSpeed = 2;
    [SerializeField] private float _cubeXStartPos;
    [SerializeField] private float _cubeYStartPos;
    [SerializeField] private float _cubeXEndPos;
    [SerializeField] private float _cubeYEndPos;
    private float2 _cubeScale => new float2(_cube.transform.localScale.x, _cube.transform.localScale.y);
    private bool _begin;
    private float2 _endPos;

    private List<GameObject> _cubes = new List<GameObject>();


    private void SimulationArea(int x, int y, int boxX = 8, int boxY = 8)
    {
        const float spacing = 1.0f;

        for (float i = -boxX / 2f; i < boxX / 2f; i += spacing)
        {
            for (float j = -boxY / 2f; j < boxY / 2f; j += spacing)
            {
                
                var pos = math.float2(x + i, y + j);
                _tempPositions.Add(pos);
                var go = Instantiate(_spawnCube, new Vector3(pos.x, pos.y, 0), Quaternion.identity);
                _cubes.Add(go);
            }
        }
    }

    private void Start()
    {
        _endPos = new float2(GridRes / _cubeXEndPos, GridRes / _cubeYEndPos);
        _cubeParticle.Position = new float2(GridRes / _cubeXStartPos, GridRes / _cubeYStartPos);
        _cubeParticle.Velocity = _endPos - _cubeParticle.Position;
        _cubeParticle.Mass = _cubeMass;
        _cube.transform.position = new Vector3(_cubeParticle.Position.x, _cubeParticle.Position.y, 0);
        
        // 1. initialise your grid - fill your grid array with (grid_res * grid_res) cells.

        _grid = new NativeArray<Cell>(NumCells, Allocator.Persistent);

        for (int i = 0; i < NumCells; ++i)
        {
            var cell = new Cell { Velocity = 0 };
            _grid[i] = cell;
        }

        // 2. create a bunch of particles. Set their positions somewhere in the simulation domain.

        _tempPositions = new List<float2>();
        SimulationArea(GridRes / 2, GridRes / 3, GridRes / 2, GridRes / 2);

        _numParticles = _tempPositions.Count;

        _particles = new NativeArray<Particle>(_numParticles, Allocator.Persistent);

        for (int i = 0; i < _numParticles; ++i)
        {
            var p = new Particle
            {
                Position = _tempPositions[i],
                Velocity = 0,
                AffMomentMatrix = 0,
                Mass = 1.0f,
                fs = math.float2x2(1, 0, 0, 1)
            };
            //initialise deformation gradient to the identity matrix, as they're in their undeformed state
            _particles[i] = p;
        }


        // 3. optionally precompute state variables e.g. particle initial volume, if your model calls for it
        // -- begin precomputation of particle volumes
        //MPM course, equation 152

        new JobParticleToGrid()
        {
            ps = _particles,
            grid = _grid,
            numParticles = _numParticles,
            ElasticLambda = ElasticLambda,
            ElasticMu = ElasticMu,
            DT = DT,
            DX = DX,
            GridRes = GridRes
        }.Schedule().Complete();

        for (int i = 0; i < _numParticles; ++i)
        {
            var p = _particles[i];

            //quadratic interpolation weights
            //I don't know what this is actually doing in reality

            float2 cellIdx = math.floor(p.Position);
            float2 cellDiff = (p.Position - cellIdx) - 0.5f;

            //this calculation is what I don't understand
            _weights[0] = 0.5f * math.pow(0.5f - cellDiff, 2);
            _weights[1] = 0.75f - math.pow(cellDiff, 2);
            _weights[2] = 0.5f * math.pow(0.5f + cellDiff, 2);

            // weights calculated for position 0, 1, and 2
            // 2D grid of weights calculated against main position
            // This is shown as:
            //      | 0.x * 0.y | 0.x * 1.y | 0.x * 2.y |
            //      | 1.x * 0.y | 1.x * 1.y | 1.x * 2.y |
            //      | 2.x * 0.y | 2.x * 1.y | 2.x * 2.y |

            // The corresponding cells at these positions are identified and density is calculated
            // As each cell contributes to the density of the current cell using the total weight and the corresponding cell mass

            float density = 0.0f;

            for (int gx = 0; gx < 3; ++gx)
            {
                for (int gy = 0; gy < 3; ++gy)
                {
                    float weight = _weights[gx].x * _weights[gy].y;

                    // map 2D to 1D index in grid
                    int cellIndex = ((int) cellIdx.x + (gx - 1)) * GridRes + ((int) cellIdx.y + gy - 1);
                    density += _grid[cellIndex].Mass * weight;
                }
            }

            //per-particle volume estimate has now been computed
            float volume = p.Mass / density;
            p.Volume = volume;

            _particles[i] = p;
        }

        // -- end precomputation of particle volumes
    }

    private void Update()
    {
        //HandleMouseInteraction();
        if (Input.GetKeyDown(KeyCode.Space)) _begin = true;

        for (int i = 0; i < Iterations; ++i)
        {
            EachSimulationStep();
        }

        RenderFrame(_particles);
    }

    //Move the cube positions by the particle positions
    //Should check for a more performant way of handling this
    public void RenderFrame(NativeArray<Particle> ps)
    {

        for (int i = 0; i < ps.Length; i++)
        {
            var parPos = ps[i].Position;
            _cubes[i].transform.position = new Vector3(parPos.x, parPos.y, 0);
        }

        if (!_begin) return;
        _cubeParticle.Velocity = _endPos - _cubeParticle.Position;

        _cubeParticle.Position += _cubeParticle.Velocity * ((DT / DX) * _cubeSpeed);

        _cube.transform.position = new Vector3(_cubeParticle.Position.x, _cubeParticle.Position.y, 0);
    }

    private void EachSimulationStep()
    {
        // 1. reset grid completely
        Profiler.BeginSample("Clear Grid");
        new JobClearGrid()
        {
            grid = _grid
        }.Schedule(NumCells, Division).Complete();
        Profiler.EndSample();

        // 2. particle-to-grid
        // goal: transfers data from particles to our grid
        Profiler.BeginSample("Particle To Grid");
        new JobParticleToGrid()
        {
            ps = _particles,
            grid = _grid,
            numParticles = _numParticles,
            ElasticLambda = ElasticLambda,
            ElasticMu = ElasticMu,
            DT = DT,
            DX = DX,
            GridRes = GridRes
        }.Schedule().Complete();
        Profiler.EndSample();

        // 3. calculate grid velocities
        Profiler.BeginSample("Update Grid Velocity");
        new JobUpdateGridVelocity()
        {
            grid = _grid,
            DT = DT,
            Gravity = Gravity,
            GridRes = GridRes
        }.Schedule(NumCells, Division).Complete();
        Profiler.EndSample();
        
        if (_begin)
        {
            // Grid-Based Collision
            Profiler.BeginSample("Grid-Based Collision");
            new GridBodyCollision
            {
                grid = _grid,
                particle = _cubeParticle,
                DT = DT,
                DX = DX,
                speed = _cubeSpeed,
                endPos = _endPos,
                scale = _cubeScale / 2,
                GridRes = GridRes
            }.Schedule().Complete();
            Profiler.EndSample();
        }
        

        // 4. grid-to-particle
        // goal: report our grid's findings back to our particles, and integrate their position + velocity forward
        Profiler.BeginSample("Grid to Particle");
        new JobGridToParticle()
        {
            ps = _particles,
            grid = _grid,
            DT = DT,
            GridRes = GridRes
        }.Schedule(_numParticles, Division).Complete();
        Profiler.EndSample();
    }

    #region Jobs

    [BurstCompile]
    struct JobClearGrid : IJobParallelFor
    {
        public NativeArray<Cell> grid;

        public void Execute(int index)
        {
            var cell = grid[index];
            // zero out mass and velocity for this cell
            cell.Mass = 0;
            cell.Velocity = 0;

            grid[index] = cell;
        }
    }

    [BurstCompile]
    unsafe struct JobParticleToGrid : IJob
    {
        public NativeArray<Cell> grid;
        [ReadOnly] public NativeArray<Particle> ps;
        [ReadOnly] public int numParticles;
        public float ElasticLambda;
        public float ElasticMu;
        public float DT;
        public float DX;
        public int GridRes;

        public void Execute()
        {
            var weights = stackalloc float2[3];
            for (int i = 0; i < numParticles; ++i)
            {
                var p = ps[i];

                // 2.1: calculate quantities like e.g. stress based on constitutive equation
                float2x2 stress = 0;

                // deformation gradient
                var F = p.fs;
                var J = math.determinant(F);

                //MPM course, page 46
                var volume = p.Volume * J;

                // useful matrices for Neo-Hookean model
                var FT = math.transpose(F);
                var FinvT = math.inverse(FT);
                var FminusFinvT = F - FinvT;

                //MPM course equation 48
                var pTerm0 = ElasticMu * (FminusFinvT);
                var pTerm1 = ElasticLambda * math.log(J) * FinvT;
                var P = pTerm0 + pTerm1;

                // cauchy stress = (1 / det(F)) * P * FT
                //equation 38, MPM course
                stress = (1.0f / J) * math.mul(P, FT);

                // (Mp)^-1 = 4, see APIC paper and MPM course page 42
                // this term is used in MLS-MPM paper equation 16 with quadratic weights, Mp = (1/4) * (deltaX)^2
                //we multiply by dt as part of the process of fusing the momentum and force update for MLS-MPM
                var Mp = 0.25f * math.pow(DX, 2);
                Mp = math.pow(Mp, -1);
                var eq16term0 = -volume * Mp * stress * DT;

                // 2.2: calculate weights for the 3x3 neighbouring cells surrounding the particle's position
                // on the grid using an interpolation function

                // weights calculated for position 0, 1, and 2
                // 2D grid of weights calculated against main position
                // This is shown as:
                //      | 0.x * 0.y | 0.x * 1.y | 0.x * 2.y |
                //      | 1.x * 0.y | 1.x * 1.y | 1.x * 2.y |
                //      | 2.x * 0.y | 2.x * 1.y | 2.x * 2.y |

                // The corresponding cells at these positions are identified and density is calculated
                // As each cell contributes to the density of the current cell using the total weight and the corresponding cell mass

                uint2 cellIdx = (uint2) p.Position;
                float2 cellDiff = (p.Position - cellIdx) - 0.5f;
                weights[0] = 0.5f * math.pow(0.5f - cellDiff, 2);
                weights[1] = 0.75f - math.pow(cellDiff, 2);
                weights[2] = 0.5f * math.pow(0.5f + cellDiff, 2);

                // 2.3: for all neighbouring cells scatter our particle's momentum to the grid, using the cell's interpolation weight calculated in 2.2
                for (uint gx = 0; gx < 3; ++gx)
                {
                    for (uint gy = 0; gy < 3; ++gy)
                    {
                        float weight = weights[gx].x * weights[gy].y;

                        uint2 cellX = math.uint2(cellIdx.x + gx - 1, cellIdx.y + gy - 1);
                        float2 cellDist = (cellX - p.Position) + 0.5f;
                        float2 q = math.mul(p.AffMomentMatrix, cellDist);

                        //converting 2D index to 1D
                        int cellIndex = (int) cellX.x * GridRes + (int) cellX.y;
                        Cell cell = grid[cellIndex];

                        //MPM course, equation 172
                        float massContribution = weight * p.Mass;

                        //scatter mass to the grid
                        cell.Mass += massContribution;

                        // APIC P2G momentum contribution
                        cell.Velocity += massContribution * (p.Velocity + q);

                        //fused force/momentum update from MLS-MPM
                        //see MLS-MPM paper, equation listed after equation 28
                        float2 momentum = math.mul(eq16term0 * weight, cellDist);
                        cell.Velocity += momentum;

                        //total update on cell.Velocity is now:
                        //weight * (DT * M^-1 * p.volume * p.stress + p.mass * p.AffMomentMatrix)
                        //this is the fused momentum + force from MLS-MPM. However, instead of our stress being derived from the energy density,
                        //i use the weak form with cauchy stress. converted:
                        //p.volume * (dΨ/dF)(Fp)*(Fp_transposed)
                        //is equal to p.volume * σ

                        //note: currently "cell.velocity" refers to MOMENTUM, not velocity!
                        //this gets converted in the UpdateGrid step below

                        grid[cellIndex] = cell;
                    }
                }
            }
        }
    }

    [BurstCompile]
    struct JobUpdateGridVelocity : IJobParallelFor
    {
        public NativeArray<Cell> grid;
        public float DT;
        public float Gravity;
        public int GridRes;

        public void Execute(int index)
        {
            var cell = grid[index];

            if (cell.Mass > 0)
            {
                // 3.1 calculate grid velocity based on momentum found in stage 2
                cell.Velocity /= cell.Mass;
                cell.Velocity += DT * math.float2(0, Gravity);

                // 3.2: enforce boundary conditions
                int x = index / GridRes;
                int y = index % GridRes;
                if (x < 2 || x > GridRes - 3) cell.Velocity.x = 0;
                if (y < 2 || y > GridRes - 3) cell.Velocity.y = 0;

                grid[index] = cell;
            }
        }
    }

    [BurstCompile]
    unsafe struct JobGridToParticle : IJobParallelFor
    {
        public NativeArray<Particle> ps;

        //public NativeArray<Particle> os;
        [ReadOnly] public NativeArray<Cell> grid;
        public float DT;
        public int GridRes;

        public void Execute(int index)
        {
            var p = ps[index];

            //reset particle velocity, we calculate it from scratch each step using the grid
            p.Velocity = 0;

            // 4.1 update particles deformation gradient using MLS-MPM's velocity gradient estimate
            // Reference: MLS-MPM paper, Eq. 17

            // 4.2: calculate neighbouring cell weights as in step 2.1.
            // note: our particle's haven't moved on the grid at all by this point, so the weights will be identical

            // weights calculated for position 0, 1, and 2
            // 2D grid of weights calculated against main position
            // This is shown as:
            //      | 0.x * 0.y | 0.x * 1.y | 0.x * 2.y |
            //      | 1.x * 0.y | 1.x * 1.y | 1.x * 2.y |
            //      | 2.x * 0.y | 2.x * 1.y | 2.x * 2.y |

            // The corresponding cells at these positions are identified and density is calculated
            // As each cell contributes to the density of the current cell using the total weight and the corresponding cell mass

            uint2 cellIdx = (uint2) p.Position;
            float2 cellDiff = (p.Position - cellIdx) - 0.5f;
            var weights = stackalloc float2[]
            {
                0.5f * math.pow(0.5f - cellDiff, 2),
                0.75f - math.pow(cellDiff, 2),
                0.5f * math.pow(0.5f + cellDiff, 2)
            };

            // constructing affine per-particle momentum matrix from APIC / MLS-MPM.
            // see APIC paper (https://web.archive.org/web/20190427165435/https://www.math.ucla.edu/~jteran/papers/JSSTS15.pdf), page 6
            // below equation 11 for clarification. this is calculating C = B * (D^-1) for APIC equation 8,
            // where B is calculated in the inner loop at (D^-1) = 4 is a constant when using quadratic interpolation functions
            float2x2 b = 0;
            // 4.3: calculate our new particle velocities
            for (uint gx = 0; gx < 3; ++gx)
            {
                for (uint gy = 0; gy < 3; ++gy)
                {
                    // 4.3.1:
                    // get this cell's weighted contribution to our particle's new velocity
                    float weight = weights[gx].x * weights[gy].y;

                    uint2 cellX = math.uint2(cellIdx.x + gx - 1, cellIdx.y + gy - 1);
                    int cellIndex = (int) cellX.x * GridRes + (int) cellX.y;

                    float2 dist = (cellX - p.Position) + 0.5f;
                    float2 weightedVelocity = grid[cellIndex].Velocity * weight;

                    // APIC paper equation 10, constructing inner term for b
                    var term = math.float2x2(weightedVelocity * dist.x, weightedVelocity * dist.y);

                    b += term;

                    p.Velocity += weightedVelocity;
                }
            }

            p.AffMomentMatrix = b * 4;

            // 4.4: advect particle positions by their velocity
            p.Position += p.Velocity * DT;

            //safety clamp to ensure particles don't exit simulation domain
            p.Position = math.clamp(p.Position, 1, GridRes - 2);

            // boundaries
            float2 x_n = p.Position + p.Velocity;
            const float wall_min = 3;
            float wall_max = (float) GridRes - 4;
            if (x_n.x < wall_min) p.Velocity.x += wall_min - x_n.x;
            if (x_n.x > wall_max) p.Velocity.x += wall_max - x_n.x;
            if (x_n.y < wall_min) p.Velocity.y += wall_min - x_n.y;
            if (x_n.y > wall_max) p.Velocity.y += wall_max - x_n.y;

            //deformation gradient update - MPM course, equation 181
            // Fp' = (I + dt * p.AffMomentMatrix) * Fp
            var fpNew = math.float2x2(1, 0, 0, 1);
            fpNew += DT * p.AffMomentMatrix;
            p.fs = math.mul(fpNew, p.fs);

            ps[index] = p;
        }
    }

    [BurstCompile]
    struct GridBodyCollision : IJob
    {
        public NativeArray<Cell> grid;
        public Particle particle;
        public float2 endPos;
        public float2 scale;
        public float DT;
        public float DX;
        public float speed;
        public int GridRes;

        public void Execute()
        {
            var p = particle;

            for (var i = -scale.x; i <= scale.x; i++)
            {
                for (var j = -scale.y; j <= scale.y; j++)
                {
                    var boundPos = new float2(p.Position.x + i, p.Position.y + j);
                    var endBoundPos = new float2(endPos.x + i, endPos.y + j);
                    
                    uint2 cellIdx = (uint2) boundPos;
                    //converting 2D index to 1D
                    int cellIndex = (int) cellIdx.x * GridRes + (int) cellIdx.y;
                    if (cellIndex >= grid.Length || cellIndex < 0) continue;
                    Cell cell = grid[cellIndex];
                    cell.Velocity = p.Mass * (p.Velocity  * ((DT / DX) * speed));
                    grid[cellIndex] = cell;
            
                    //difference vector
                    var x = endBoundPos.x - boundPos.x;
                    var y = endBoundPos.y - boundPos.y;
            
                    //Length to normalize
                    var vecLen = (float)Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2));
            
                    //Normalise
                    x /= vecLen;
                    y /= vecLen;

                    //converting 2D index to 1D
                    cellIndex = (int) (cellIdx.x + x) * GridRes + (int) (cellIdx.y + y);
                    if (cellIndex >= grid.Length || cellIndex < 0) continue;
                    cell = grid[cellIndex];
                    cell.Velocity = p.Mass * (p.Velocity * ((DT / DX) * speed));
                    grid[cellIndex] = cell;
                }
            }
            
            particle = p;
        }
    }

    #endregion

    private void OnDestroy()
    {
        _particles.Dispose();
        _grid.Dispose();
    }
}
