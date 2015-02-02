# Irradiance Importance Sampling
Bachelor Thesis of Alisa Jung

This is the repository of my Bachelor Thesis. Among other things, it contains the Thesis, the Mitsuba Renderer including my new integrator, several test scenes and images.

<h2>Main Idea</h2>

The main idea of this work is to use caches for incident radiance (IRCs) to importance-sample irradiance. The IRCs are used in a path tracer to reduce the variance of Multiple Importance Sampling.
<p>The caches are created in a preprocessing step. First comes a photon tracing pass, the number of photon paths can be specified in the GUI. Next, the caches are created, some of them at random photon positions and some of them from camera rays. Each cache is represented by an environment map. The texels of the environment map are then filled with an approximation of the incident radiance of the direction they represent, using the photons from the photon map. At the end, the photon map can be deleted. Each cache contains an approximation of the incident radiance over different directions. The number of caches and number of photons used to fill a cache can be varied through the GUI.
<p>The IRCs can be used instead of BSDF sampling to extend paths. The current implementation applies them to a regular path tracer, but including IRCs to a bidirectional path tracer should be possible as well. Whenever a surface point is hit, the closest IRC to that point is queried, and a sample is generated from the IRC. The sampled direction is proportional to the radiance values stored in the texels. Thus the environment map contains a probability density function that is roughly proportional to the irradiance. The goal of this thesis was to evaluate whether this approach to importance sampling the irradiance could reduce the variance in the final image.




<h2>Code</h2>
I used the Mitsuba Renderer as a framework. You can get it here: <a href="https://www.mitsuba-renderer.org/">https://www.mitsuba-renderer.org/</a>. Mitsuba already provides several rendering algorithms (e.g. path tracing, bidirectional path tracing, photon mapping, ...) by default, but allows to include own techniques. It also takes care of all kinds of things like scene description, different surface properties, multithreading, and creating the initial rays for a camera.
<p>Our integrator (Mitsuba's term for rendering techniques) is called ```MyPathTracerAndPhotonMapper.cpp``` and can be found in mitsuba/src/integrators. Some additional files were altered so the integrator is compiled correctly, and so its parameters can be modified over the GUI.

<h5>GUI Properties</h5>
The goal of this thesis was to evaluate whether incident radiance caches could improve images in comparison to conventional path tracing with BSDF sampling and next event estimation (also known as sampling the light source). The GUI allows for different settings that combine IRC sampling, BSDF sampling and NEE.

Settings  | Effect
------------- | -------------
fix w_I = -1, fix w_E = -1, sample caches only = true  | IRC sampling combined with NEE
fix w_I = -1, fix w_E = -1, sample bsdf only = true | BSDF sampling combined with NEE
fix w_I = 100, fix w_E = 0 | BSDF sampling combined with IRC sampling

Note that any other combination is not completely consistent in the code. Especially the combination of all three techniques (BSDF sampling, IRC sampling and NEE) was not implemented, as some additional attention has to be paid to the MIS weights in that case.

<h5>Recommended Settings</h5>
For the provided test scenes, the number of photon paths should be at least 100k and the number of photons per cache at least 100 in order to avoid unnecessary noise. The preprocessing step only uses a small fraction of the total computation time anyway, so increasing these values doesn't really affect the total rendering time.
<p>The camera caches are created once for every 8x8 pixel block by default. This can only be changed in the code and they are created even if their number exceeds the specified number of caches. All remaining caches are created from Photons. Only using camera caches can cause artefacts along edges. The cornell box scenes end up with ~3800 camera caches, 10k - 20k caches in total yield good results.


<h2>Abstract</h2>
For many years path tracing has been a popular choice when it came to rendering photorealistic images. By combining different sampling techniques, the algorithm can be adapted to the complexity and challenges of individual scenes. That way eects like indirect lighting and caustics can be rendered realistically.
This work will introduce a new approach to include irradiance importance sampling to a regular path tracing algorithm. Before an image is rendered, the incident radiance (direct and indirect) across a scene is approximated and cached. The information from these caches can be used to generate paths with a potentially high contribution to the rendering equation.
Caching and importance sampling the incident radiance enables us to compensate for some of the drawbacks of sampling paths solely according to local surface properties. This approach may also be combined with known sampling techniques, such as next event estimation or generating samples from a BSDF (bidirectional scattering distribution function), using multiple importance sampling.




