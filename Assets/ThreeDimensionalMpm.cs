using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;

public class ThreeDimensionalMpm : MonoBehaviour
{
    //TODO: Expand code to 3 Dimensional calculations
    public struct Particle {
        public float3 Position;
        public float3 Velocity;
        public float3x3 AffMomentMatrix;
        public float Mass;
        public float Volume;
    }

    private struct Cell {
        public float3 Velocity;
        public float Mass;
    }

    private const int GridRes = 16;
    private const int NumCells = GridRes * GridRes * GridRes;
    
    // batch size for the job system.
    private const int Division = 16;
    
    //simulation parameters

    //dt = the time step of the simulation. The stability of the simulation is going to be limited by how much a particle can move in a single time step
    //it's a good rule of thumb to choose dt so that no particle could move more than 1 grid-cell in a single step
    private const float DT = 0.05f;
    private const int Iterations = (int)(1.0f / DT);
    private const float Gravity = -0.3f;
    
    // Lamé parameters for stress-strain relationship
    private const float ElasticLambda = 0f;
    private const float ElasticMu = 100f;

    private int _numParticles;

    private NativeArray<Particle> _particles;
    private NativeArray<Cell> _grid;
    
    //Deformation gradient, can be in particle struct. Here for rendering code (not mine)
    private NativeArray<float3x3> Fs;

    private float3[] _weights = new float3[3];

    private List<float3> _tempPositions;

    [SerializeField] private GameObject _spawnCube;
    [SerializeField] private GameObject _cube;

    private List<GameObject> _cubes = new List<GameObject>();

    private void SimulationArea(int x, int y, int z, int boxX = 8, int boxY = 8, int boxZ = 8)
    {
        const float spacing = 1.0f;

        for (float i = - boxX / 2; i < boxX / 2; i += spacing)
        {
            for (float j = - boxY / 2; j < boxY / 2; j += spacing)
            {
                for (float k = - boxZ / 2; k < boxZ / 2; k += spacing)
                {
                    var pos = math.float3(x + i, y + j, z + k);
                    _tempPositions.Add(pos);
                    _cubes.Add(Instantiate(_spawnCube, new Vector3(pos.x, pos.y, pos.z), Quaternion.identity));
                }
            }
        }
    }

    private void Start(){
        // 1. initialise your grid - fill your grid array with (grid_res * grid_res) cells.
        
        _grid = new NativeArray<Cell>(NumCells, Allocator.Persistent);

        for (int i = 0; i < NumCells; ++i)
        {
            var cell = new Cell();
            cell.Velocity = 0;
            _grid[i] = cell;
        }

        // 2. create a bunch of particles. Set their positions somewhere in your simulation domain.
        // initialise their deformation gradients to the identity matrix, as they're in their undeformed state.
        
        _tempPositions = new List<float3>();
        SimulationArea(GridRes / 2, GridRes / 2, GridRes / 2);
        
        _numParticles = _tempPositions.Count;

        _particles = new NativeArray<Particle>(_numParticles, Allocator.Persistent);
        Fs = new NativeArray<float3x3>(_numParticles, Allocator.Persistent);
        
        for (int i = 0; i < _numParticles; ++i)
        {
            Particle p = new Particle();
            p.Position = _tempPositions[i];
            p.Velocity = 0;
            p.AffMomentMatrix = 0;
            p.Mass = 1.0f;
            _particles[i] = p;
            
            // deformation gradient initialised to the identity
            Fs[i] = math.float3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
        }

        // 3. optionally precompute state variables e.g. particle initial volume, if your model calls for it
        // -- begin precomputation of particle volumes
        //MPM course, equation 152
        
        new JobParticleToGrid()
        {
            ps = _particles,
            fs = Fs,
            grid = _grid,
            numParticles = _numParticles
        }.Schedule().Complete();

        for (int i = 0; i < _numParticles; ++i)
        {
            var p = _particles[i];
            
            //quadratic interpolation weights
            float3 cellIdx = math.floor(p.Position);
            float3 cellDiff = (p.Position - cellIdx) - 0.5f;
            _weights[0] = 0.5f * math.pow(0.5f - cellDiff, 2);
            _weights[1] = 0.75f - math.pow(cellDiff, 2);
            _weights[2] = 0.5f * math.pow(0.5f + cellDiff, 2);
            
            float density = 0.0f;

            for (int gx = 0; gx < 3; ++gx)
            {
                for (int gy = 0; gy < 3; ++gy)
                {
                    for (int gz = 0; gz < 3; gz++)
                    {
                        float weight = _weights[gx].x * _weights[gy].y * _weights[gz].z;

                        // map 2D to 1D index in grid
                        int cellIndex = ((int) cellIdx.x + GridRes * ((int) cellIdx.y + GridRes * (int) cellIdx.z));
                        density += _grid[cellIndex].Mass * weight;
                    }
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
        for (int i = 0; i < Iterations; ++i)
        {
            EachSimulationStep();
        }
        
        RenderFrame(_particles);
    }
    
    public void RenderFrame(NativeArray<Particle> ps) {

        for (int i = 0; i < ps.Length; i++)
        {
            var parPos = ps[i].Position;
            _cubes[i].transform.position = new Vector3(parPos.x, parPos.y, parPos.z);
        }
    }

    private void EachSimulationStep() {
        // 1. reset our scratch-pad grid completely
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
            fs = Fs,
            grid = _grid,
            numParticles = _numParticles
        }.Schedule().Complete();
        Profiler.EndSample();
        
        // 3. calculate grid velocities
        Profiler.BeginSample("Update Grid");
        new JobUpdateGrid()
        {
            grid = _grid
        }.Schedule(NumCells, Division).Complete();
        Profiler.EndSample();

        var position = _cube.transform.position;
        var cubePos = math.float3(position.x, position.y, position.z);

        // 4. grid-to-particle
        // goal: report our grid's findings back to our particles, and integrate their position + velocity forward
        Profiler.BeginSample("Grid to Particle");
        new JobGridToParticle()
        {
            ps = _particles,
            fs = Fs,
            grid = _grid,
            cube = cubePos
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
        [ReadOnly] public NativeArray<float3x3> fs;
        [ReadOnly] public int numParticles;

        public void Execute()
        {
            var weights = stackalloc float3[3];
            for (int i = 0; i < numParticles; ++i)
            {
                var p = ps[i];
                
                // 2.1: calculate quantities like e.g. stress based on constitutive equation
                float3x3 stress = 0;
                
                // deformation gradient
                var F = fs[i];
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
                //in this simulation, deltaX = 1, because i scale the rendering of the domain rather than the domain itself
                //we multiply by dt as part of the process of fusing the momentum and force update for MLS-MPM
                var eq16term0 = -volume * 4 * stress * DT;
                
                // // 2.2: calculate weights for the 3x3 neighbouring cells surrounding the particle's position
                // on the grid using an interpolation function

                uint3 cellIdx = (uint3)p.Position;
                float3 cellDiff = (p.Position - cellIdx) - 0.5f;
                weights[0] = 0.5f * math.pow(0.5f - cellDiff, 3);
                weights[1] = 0.75f - math.pow(cellDiff, 3);
                weights[2] = 0.5f * math.pow(0.5f + cellDiff, 3);

                //2.3: for all neighbouring cells scatter our particle's momentum to the grid, using the cell's interpolation weight calculated in 2.1
                for (uint gx = 0; gx < 3; ++gx)
                {
                    for (uint gy = 0; gy < 3; ++gy)
                    {
                        for (uint gz = 0; gz < 3; gz++)
                        {
                            float weight = weights[gx].x * weights[gy].y * weights[gz].z;

                            uint3 cellX = math.uint3(cellIdx.x + gx - 1, cellIdx.y + gy - 1, cellIdx.z + gz - 1);
                            float3 cellDist = (cellX - p.Position) + 0.5f;
                            float3 q = math.mul(p.AffMomentMatrix, cellDist);

                            //converting 2D index to 1D
                            int cellIndex = (int) cellIdx.x + GridRes * ((int)cellIdx.y + GridRes * (int)cellIdx.z);
                            Cell cell = grid[cellIndex];

                            //MPM course, equation 172
                            float massContribution = weight * p.Mass;

                            //scatter mass to the grid
                            cell.Mass += massContribution;

                            // APIC P2G momentum contribution
                            cell.Velocity += massContribution * (p.Velocity + q);

                            //fused force/momentum update from MLS-MPM
                            //see MLS-MPM paper, equation listed after equation 28
                            float3 momentum = math.mul(eq16term0 * weight, cellDist);
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
    }

    [BurstCompile]
    struct JobUpdateGrid : IJobParallelFor
    {
        public NativeArray<Cell> grid;

        public void Execute(int index)
        {
            var cell = grid[index];
            
            if (cell.Mass > 0)
            {
                // 3.1 calculate grid velocity based on momentum found in stage 2
                cell.Velocity /= cell.Mass;
                cell.Velocity += DT * math.float3(0, Gravity, 0);
                
                // 3.2: enforce boundary conditions
                int x = index / GridRes;
                int y = index % GridRes;
                int z = index / (GridRes * GridRes);
                if (x < 2 || x > GridRes - 3) cell.Velocity.x = 0;
                if (y < 2 || y > GridRes - 3) cell.Velocity.y = 0;
                if (z < 2 || z > GridRes - 3) cell.Velocity.z = 0;

                grid[index] = cell;
            }
        }
    }

    unsafe struct JobGridToParticle : IJobParallelFor
    {
        public NativeArray<Particle> ps;
        public NativeArray<float3x3> fs;
        [ReadOnly] public NativeArray<Cell> grid;
        public float3 cube;

        public void Execute(int index)
        {
            var p = ps[index];

            //reset particle velocity, we calculate it from scratch each step using the grid
            p.Velocity = 0;

            // 4.1 update particles deformation gradient using MLS-MPM's velocity gradient estimate
            // Reference: MLS-MPM paper, Eq. 17

            // 4.2: calculate neighbouring cell weights as in step 2.1.
            // note: our particle's haven't moved on the grid at all by this point, so the weights will be identical
            uint3 cellIdx = (uint3) p.Position;
            float3 cellDiff = (p.Position - cellIdx) - 0.5f;
            var weights = stackalloc float3[]
            {
                0.5f * math.pow(0.5f - cellDiff, 2),
                0.75f - math.pow(cellDiff, 2),
                0.5f * math.pow(0.5f + cellDiff, 2)
            };

            // constructing affine per-particle momentum matrix from APIC / MLS-MPM.
            // see APIC paper (https://web.archive.org/web/20190427165435/https://www.math.ucla.edu/~jteran/papers/JSSTS15.pdf), page 6
            // below equation 11 for clarification. this is calculating C = B * (D^-1) for APIC equation 8,
            // where B is calculated in the inner loop at (D^-1) = 4 is a constant when using quadratic interpolation functions
            float3x3 b = 0;
            // 4.3: calculate our new particle velocities
            for (uint gx = 0; gx < 3; ++gx)
            {
                for (uint gy = 0; gy < 3; ++gy)
                {
                    for (uint gz = 0; gz < 3; gz++)
                    {
                        // 4.3.1:
                        // get this cell's weighted contribution to our particle's new velocity
                        float weight = weights[gx].x * weights[gy].y * weights[gz].z;

                        uint3 cellX = math.uint3(cellIdx.x + gx - 1, cellIdx.y + gy - 1, cellIdx.z + gz - 1);
                        int cellIndex = (int) cellIdx.x + GridRes * ((int)cellIdx.y + GridRes * (int)cellIdx.z);

                        float3 dist = (cellX - p.Position) + 0.5f;
                        float3 weightedVelocity = grid[cellIndex].Velocity * weight;

                        // APIC paper equation 10, constructing inner term for b
                        var term = math.float3x3(weightedVelocity * dist.x, weightedVelocity * dist.y, weightedVelocity * dist.z);

                        b += term;

                        p.Velocity += weightedVelocity;
                    }
                }
            }

            p.AffMomentMatrix = b * 4;

            // 4.4: advect particle positions by their velocity
            p.Position += p.Velocity * DT;

            //safety clamp to ensure particles don't exit simulation domain
            p.Position = math.clamp(p.Position, 1, GridRes - 2);

            //mouse interaction
            // if (mouseDown)
            // {
            //     var dist = p.Position - mousePos;
            //     if (math.dot(dist, dist) < MouseRadius * MouseRadius)
            //     {
            //         float normFactor = (math.length(dist) / MouseRadius);
            //         normFactor = math.pow(math.sqrt(normFactor), 8);
            //         var force = math.normalize(dist) * normFactor * 0.5f;
            //         p.Velocity += force;
            //     }
            // }
            
            var cubeDist = p.Position - cube;
            var cubeRadius = 2f;
            if (math.dot(cubeDist, cubeDist) < cubeRadius * cubeRadius)
            {
                float normFactor = (math.length(cubeDist));
                normFactor = math.pow(math.sqrt(normFactor), 8);
                var force = math.normalize(cubeDist) * normFactor * 0.5f;
                p.Velocity += force;
            }


            //deformation gradient update - MPM course, equation 181
            // Fp' = (I + dt * p.AffMomentMatrix) * Fp
            var FpNew = math.float3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
            FpNew += DT * p.AffMomentMatrix;
            fs[index] = math.mul(FpNew, fs[index]);

            ps[index] = p;
        }
    }
    
    #endregion

    private void OnDestroy()
    {
        _particles.Dispose();
        _grid.Dispose();
        Fs.Dispose();
    }
}
