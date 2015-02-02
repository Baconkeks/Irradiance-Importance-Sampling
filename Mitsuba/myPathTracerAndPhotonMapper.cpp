#include <mitsuba/render/scene.h>
#include <mitsuba/nnpointcloud.h>
#include <mitsuba/nnkdtree.h>
#include <mitsuba/myenvmap.h>
#include <ctime>
#include <array>
#include <random>
#include <functional>

MTS_NAMESPACE_BEGIN

struct Photon{
	Point position;
	Frame surface_normal_frame; //Jaja normale braucht weniger platz. später.
	float energy_avg;
	const BSDF* bsdf;
	Vector w_i; //im Moment in Weltkoordinaten

	Photon(Point position, Frame normal, float energy_avg, const BSDF* bsdf, Vector w_i){
		this->position = position;
		this->surface_normal_frame = normal;
		this->energy_avg = energy_avg;
		this->bsdf = bsdf;
		this->w_i = w_i;
	}
};

struct LocalEnvMap{
public:
	static const int size_oneHemisphere = 256;
	static const int width_oneHemisphere = 16;
	static const int height_oneHemisphere = 16;

	LocalEnvMap(bool bothHemispheres, Frame f){
		frame = f;
		usesBothHemispheres = bothHemispheres;
		if (usesBothHemispheres){
			size = 2 * size_oneHemisphere;
			width = 2 * width_oneHemisphere;;
			height = height_oneHemisphere;
		}
		else {
			size = size_oneHemisphere;
			width = width_oneHemisphere;
			height = height_oneHemisphere;
		}
	}

	int getSize(){
		return size;
	}

	int getHeight(){
		return height;
	}

	int getWidth(){
		return width;
	}

	Normal getNormal(){
		return frame.n;
	}

	float pdf_unitsquare_to_solidangle(float pdf){
		if (usesBothHemispheres) return (pdf*size) / (4.0f*M_PI_FLT);
		else return (pdf*size) / (2.0f*M_PI_FLT);
	}

	void setEnvMap(std::vector<float> envmap, std::vector<float> solidAngles){
		assert(envmap.size() == size);
		//if (!usesBothHemispheres) assert(solidAngleMap.size() == size);
		envmap_pdf = envmap;
		envmap_cdf.resize(size);
		float sum = 0;
		for (int i = 0; i < size; i++){
			sum += envmap_pdf[i];
			envmap_cdf[i] = sum;
			envmap_pdf[i] /= solidAngles[i];
		}
	}

	void storeEnvMap_pdf(std::vector<float> envmap){
		envmap_pdf = envmap;
	}

	std::vector<float> getEnvmap_pdf(){
		return envmap_pdf;
	}

	bool doesUseBothHemispheres(){
		return usesBothHemispheres;
	}

	/*Texturkoordinate aus [0,1]^2 -> Richtung mit y oben*/
	Vector3 textureToDirection(Point2d texel){
		Vector d;
		Point2d p = texel;
		if (usesBothHemispheres){
			p.x = 4 * p.x - 2;
		}
		else {
			p.x = 2 * p.x - 2;
			assert(p.x <= 0);
		}
		p.y = 2 * p.y - 1;
		if (p.x <= 0){
			d[0] = 0.5f*(p.x + p.y + 1);
			d[2] = p.y - d[0];
			d[1] = 1 - std::abs(d[0]) - std::abs(d[2]);
		}
		else {
			d[0] = 0.5f*(-p.x + p.y + 1);
			d[2] = p.y - d[0];
			d[1] = -1 + std::abs(d[0]) + std::abs(d[2]);
		}
		return d;
	}

	Vector3 localDirToGlobalDir(Vector3 d){
		return frame.toWorld(d);
	}

	Vector3 toLocal(Vector3 d){
		return frame.toLocal(d);
	}

	float cosTheta(Vector3 d){
		return frame.cosTheta(d);
	}

	/*Richtung mit y oben -> Texturkoordinate aus [0,1]^2*/
	Point2d directionToTexture(Vector3 d){
		Vector p_ = d / (std::abs(d[0]) + std::abs(d[1]) + std::abs(d[2])); //planare projektion
		Point2d p;
		p.y = p_[0] + p_[2];
		if (p_[1] >= 0){
			p.x = p_[0] - p_[2] - 1;
		}
		else {
			p.x = p_[2] - p_[0] + 1;
		}
		//Skaliere, sodass Ursprung unten links und p\in [0,1]^2
		if (usesBothHemispheres){
			p.x += 2;
			p.x *= 0.25f;
		}
		else {
			assert(p_[1] >= 0);
			p.x += 2;
			p.x *= 0.5f;
		}
		p.y += 1;
		p.y *= 0.5f;
		return p;
	}

	/*	- Binäre Suche in envmapcache_cdf nach erstem Feld, das >= randomnumber ist
	- gibt als pdf den Wert in envmapcache an diesem Feld zurück
	- Bestimme Texel (ganzzahlig)
	- Bestimme Punkt in Texel mit texelOffset
	- Skaliere Texel auf [0,1]^2*/
	Point2d sampleLocalEnvMapTexel(float& pdf, float randomnumber, Point2 texelOffset){
		/*Binäre Suche*/
		int index = size / 2;
		bool found = false;
		int leftIndex = 0;
		int rightIndex = size - 1;
		int mitte;
		while (leftIndex < rightIndex){
			mitte = leftIndex + ((rightIndex - leftIndex)) / 2;
			if (envmap_cdf[mitte] < randomnumber){
				leftIndex = mitte + 1;
			}
			else {
				rightIndex = mitte;
			}
		}
		assert(rightIndex == leftIndex);
		double x = ((rightIndex%width) + texelOffset.x) / width; //Texel + zufälliges x in diesem Texel, skaliert auf (0,1)
		double y = ((rightIndex / width) + texelOffset.y) / height;//Texel.y + zufälliges y in diesem Texel, skaliert auf (0,1)

		pdf = (float)envmap_pdf[rightIndex];
		return Point2d(x, y);
	}

	/*Nimmt lokale Richtung in mitsuba-Koordinaten (z oben) und bestimmt passende Wahrscheinlichkeit*/
	float getProbabilityForEnvmapDirection(Vector dir){
		//Mit welcher Wahrscheinlichkeit hätte man in der envmap die Richtung dir gesamplet?
		Vector dirInTexelCoords = Vector(dir.x, dir.z, dir.y);
		assert(usesBothHemispheres || dirInTexelCoords.y >= 0);
		Point2d texel = directionToTexture(dirInTexelCoords);
		int x = (int)(floor(texel.x * width));
		int y = (int)(floor(texel.y*height));
		return (float)envmap_pdf[x + y*width];
	}

private:
	int size;
	int width;
	int height;

	Frame frame;

	bool usesBothHemispheres;
	std::vector<float> envmap_pdf;
	std::vector<float> envmap_cdf;
};

class MyPathTracerAndPhotonMapper : public	MonteCarloIntegrator
{
public:
	MyPathTracerAndPhotonMapper(const Properties &props) : MonteCarloIntegrator(props)
	{
		m_show_caches = props.getBoolean("show_caches", false);

		m_traced_photon_paths = props.getInteger("traced_photon_paths", 1);
		m_number_of_envmaps = props.getInteger("number_of_envmaps", 10000);

		m_fix_w_I = props.getFloat("fix_w_I", -1.0f) / 100;
		m_fix_w_E = props.getFloat("fix_w_E", -1.0f) / 100;

		m_use_fix_w_I = (m_fix_w_I >= -0.00001);
		m_use_fix_w_E = (m_fix_w_E >= -0.00001);

		m_rrDepth = props.getInteger("rrDepth", 32);
		m_use_own_envmap = props.getBoolean("use_own_envmap", false);
		if (m_use_own_envmap){
			m_envmap = new MyEnvMap();
		}
		m_sample_BSDF_only = props.getBoolean("sample_BSDF", false);
		m_sample_cache_only = props.getBoolean("sample_cache_only", false);

		std::srand(std::time(0));
		m_photons.reserve(8 * m_traced_photon_paths);
		m_pointCloud_photons.points.reserve(8 * m_traced_photon_paths);
		m_envmaps.reserve(m_number_of_envmaps);
		m_pointCloud_envmaps.points.reserve(m_number_of_envmaps);

		//m_number_of_envmaps /= m_traced_photon_paths; //in jedem Durchlauf von 100k Photonen werden ein teil der envmaps gefüllt
		m_samples_for_envmap = props.getInteger("samples_for_envmap", 500); //In jedem Durchlauf wird jede Envmap nur anteilig befüllt
		m_envmaps_available = false;

		gen = std::mt19937(time(0));
		dis = std::uniform_real_distribution<double>(0, 1);

		Log(EDebug, "traced_photon_paths = %d, number_of_envmaps = %d, fix_w_I = %f, fix_w_E = %f, useFixwE %d, sample_BSDF = %d, samples_for_envmaps %i", m_traced_photon_paths, m_number_of_envmaps, m_fix_w_I, m_fix_w_E, m_use_fix_w_E, m_sample_BSDF_only, m_samples_for_envmap);
	}

	MyPathTracerAndPhotonMapper(Stream *stream, InstanceManager *manager) : MonteCarloIntegrator(stream, manager){
		m_show_caches = stream->readBool();
		m_traced_photon_paths = stream->readInt();
		m_number_of_envmaps = stream->readInt();

		m_fix_w_I = stream->readFloat();
		m_fix_w_E = stream->readFloat();
		m_use_own_envmap = stream->readBool();
		m_sample_BSDF_only = stream->readBool();
		m_sample_cache_only = stream->readBool();

		m_samples_for_envmap = stream->readInt();
	}

	void serialize(Stream *stream, InstanceManager *manager) const{
		SamplingIntegrator::serialize(stream, manager);
		stream->writeBool(m_show_caches);
		stream->writeInt(m_traced_photon_paths);
		stream->writeInt(m_number_of_envmaps);

		stream->writeFloat(m_fix_w_I);
		stream->writeFloat(m_fix_w_E);
		stream->writeBool(m_use_own_envmap);
		stream->writeBool(m_sample_BSDF_only);
		stream->writeBool(m_sample_cache_only);

		stream->writeInt(m_samples_for_envmap);
	}

	bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int sensorResID, int samplerResID){
		/*Envmap Test*/
		/*
		LocalEnvMap envmapcache;
		envmapcache.fill(0);
		double sumtest = 0;
		for (int i = 0; i < envmapcache.size(); i++){
		if (i % localEnvMapWidth < 8){
		envmapcache[i] = (double)rand() / (double)RAND_MAX;
		sumtest += envmapcache[i];
		}
		}
		double newsumtest = 0;
		for (int i = 0; i < envmapcache.size(); i++){
		envmapcache[i] /= sumtest;
		newsumtest += envmapcache[i];
		}

		Log(EDebug, "Test envmap : newsum: %f, sum: %f", newsumtest, sumtest);
		for (int a = 0; a < 8; a++){
		Log(EDebug, "Values: %f %f %f %f %f %f %f %f | %f %f %f %f %f %f %f %f", envmapcache[a * localEnvMapWidth + 0], envmapcache[a * localEnvMapWidth + 1], envmapcache[a * localEnvMapWidth + 2], envmapcache[a * localEnvMapWidth + 3], envmapcache[a * localEnvMapWidth + 4], envmapcache[a * localEnvMapWidth + 5], envmapcache[a * localEnvMapWidth + 6], envmapcache[a * localEnvMapWidth + 7], envmapcache[a * localEnvMapWidth + 8], envmapcache[a * localEnvMapWidth + 9], envmapcache[a * localEnvMapWidth + 10], envmapcache[a * localEnvMapWidth + 11], envmapcache[a * localEnvMapWidth + 12], envmapcache[a * localEnvMapWidth + 13], envmapcache[a * localEnvMapWidth + 14], envmapcache[a * localEnvMapWidth + 15]);
		}

		LocalEnvMap envmapCalledSum;
		envmapCalledSum.fill(0);
		int samples = 1000000;
		for (int i = 0; i < samples; i++){
		double sumRandi = 0;
		double randomnumber = (double)((double)rand() / (RAND_MAX + 1));//Eine rand für alle oder jedes mal neu? //TODO Achtung! Alle Threads haben die selben Zufallszahlen
		bool found = false;
		int j = 0;
		while (!found && j < envmapcache.size()){
		sumRandi += envmapcache[j];
		if (randomnumber < sumRandi){//jedes mal neue rand? - Sollte schon, sonst kann rand < sum sein, das nächste feld hat p = 0, sum bleibt gleich, aber neue rand ist auf einmal kleiner.
		int x = (j%localEnvMapWidth) ; //Texel + zufälliges x in diesem Texel, skaliert auf (0,1)
		int y = (j / localEnvMapWidth) ;//Texel.y + zufälliges y in diesem Texel, skaliert auf (0,1)
		//envmapCalledSum[j] += 1;
		envmapCalledSum[x + y*localEnvMapWidth] += 1;

		found = true;
		}
		j++;
		}
		}
		for (int i = 0; i < localEnvMapSize; i++){
		envmapCalledSum[i] /= samples;
		if (envmapCalledSum[i] > 0){
		envmapcache[i] /= envmapCalledSum[i];
		envmapcache[i] -= 1;
		}
		}


		Log(EDebug, "Test envmap : pdf / relative #hits");
		for (int a = 0; a < 8; a++){
		Log(EDebug, "Values: %f %f %f %f %f %f %f %f | %f %f %f %f %f %f %f %f", envmapcache[a * 16 + 0], envmapcache[a * 16 + 1], envmapcache[a * 16 + 2], envmapcache[a * 16 + 3], envmapcache[a * 16 + 4], envmapcache[a * 16 + 5], envmapcache[a * 16 + 6], envmapcache[a * 16 + 7], envmapcache[a * 16 + 8], envmapcache[a * 16 + 9], envmapcache[a * 16 + 10], envmapcache[a * 16 + 11], envmapcache[a * 16 + 12], envmapcache[a * 16 + 13], envmapcache[a * 16 + 14], envmapcache[a * 16 + 15]);
		}
		for (int i = 0; i < localEnvMapSize; i++){
		envmapcache[i] = envmapCalledSum[i];
		}
		Log(EDebug, "Test envmap : relative #hits");
		for (int a = 0; a < 8; a++){
		Log(EDebug, "Values: %f %f %f %f %f %f %f %f | %f %f %f %f %f %f %f %f", envmapcache[a * 16 + 0], envmapcache[a * 16 + 1], envmapcache[a * 16 + 2], envmapcache[a * 16 + 3], envmapcache[a * 16 + 4], envmapcache[a * 16 + 5], envmapcache[a * 16 + 6], envmapcache[a * 16 + 7], envmapcache[a * 16 + 8], envmapcache[a * 16 + 9], envmapcache[a * 16 + 10], envmapcache[a * 16 + 11], envmapcache[a * 16 + 12], envmapcache[a * 16 + 13], envmapcache[a * 16 + 14], envmapcache[a * 16 + 15]);
		}
		*/

		MonteCarloIntegrator::preprocess(scene, queue, job, sceneResID, sensorResID, samplerResID);
		RayDifferential ray;
		Intersection its;
		m_envmaps.clear();
		m_pointCloud_envmaps.points.clear();
		m_photons.clear();
		m_pointCloud_photons.points.clear();

		bool fillCamCaches = true;
		const std::vector<float> solidAngleMap = createSolidAngleMap();
		for (int i = 0; i < m_traced_photon_paths;){
			Point2 p(dis(gen), dis(gen));
			Point2 q(dis(gen), dis(gen));

			Intersection originOnLightSource;

			//Sample Position und Richtung nacheinander.
			PositionSamplingRecord pRec;
			Spectrum power = scene->sampleEmitterPosition(pRec, p);
			const Emitter* emitter = static_cast<const Emitter *>(pRec.object);
			DirectionSamplingRecord dRec;
			Spectrum dirWeight = emitter->sampleDirection(dRec, pRec, q);
			power *= dirWeight;
			Spectrum throughput = power * pRec.pdf / M_PI_FLT; //Ja, das "/pi" kann man weglassen, aber so ist throughput das selbe wie mit eval.
			originOnLightSource.p = pRec.p;
			ray = RayDifferential(pRec.p, dRec.d, ray.time);

			/*Beobachtung: e->eval(...) == 0 für ray.d.y < 0, e->eval(...) > 0 für ray.d.y > 0
			//Beobachtung: Aus Gründen (-.-) ist e->eval = power * pRec.pdf / pi (für ray.d.y > 0, sonst ist e->eval = 0 und power was anständiges.)
			originOnLightSource.p = ray.o;
			Spectrum throughput = e->eval(originOnLightSource, ray.d); //TODO throughput /= wahrscheinlichkeit für Richtung

			Siehe hier:
			//Spectrum ess = emitter->eval(originOnLightSource, ray.d);
			//float earea = emitter->getShape()->getSurfaceArea();
			//if (ray.d.y >= 0 && ess[0] > 0) Log(EDebug, "power: (%f), e->eval (%f), power*pdf/pi %f", throughput[0], ess[0], throughput[0] * pRec.pdf / (M_PI_FLT));
			*/

			bool continuePath = throughput.average() > 0.00001f;
			//if (!continuePath){
			//	Log(EDebug, "Throughput from eval: (%f,%f,%f), emitterSampleSpectrum (%f,%f,%f), ray.d (%f,%f,%f), emitterPdf: %f", throughput[0], throughput[1], throughput[2], ess[0], ess[1], ess[2], ray.d.x,ray.d.y,ray.d.z, emitterPdf);
			//}
			int pathlength = 0;
			bool photonStored = false;
			while (continuePath){
				if (scene->rayIntersect(ray, its) && !its.isEmitter()){
					pathlength++;
					const BSDF* bsdf = its.getBSDF();
					/*An nicht spekularen Flächen speichern*/
					if ((bsdf->getType() & BSDF::ESmooth) && !its.isEmitter()){
						photonStored = true;
						float energy_avg = throughput.average(); //Anscheinend muss man vorher den avg als float speichern, bevor er in das Photon gegeben wird. Sonst wird in dem Photon als avg 0 gespeichert (außer man speichert noch das spectrum mit)
						Photon* p = new Photon(its.p, its.geoFrame, energy_avg, its.getBSDF(), -ray.d);
						m_photons.push_back(*p);
						Vector3 v(p->position[0], p->position[1], p->position[2]);
						m_pointCloud_photons.points.push_back(v);
					}

					/*Roussian Roulette / Terminate when pathLength > 32*/
					if (pathlength > 32){
						continuePath = false;
					}
					else if (pathlength > 16){
						Spectrum diffRef = bsdf->getDiffuseReflectance(its);
						Spectrum specRef = bsdf->getSpecularReflectance(its);
						float alpha = diffRef.average() + specRef.average();
						if (alpha > 1) alpha = 1;
						float random = dis(gen);
						if (random >= alpha){
							continuePath = false;
						}
						else {
							assert(alpha > 0);
							throughput /= alpha;
						}
					}

					/*Continue Path*/
					if (continuePath){
						BSDFSamplingRecord bRec(its, -ray.d, EImportance);
						const Point2 samplePoint(dis(gen), dis(gen));
						Spectrum bsdfWeight = bsdf->sample(bRec, samplePoint);
						throughput *= bsdfWeight;
						if (throughput.average() > 0.00001f){
							const Vector wo = its.toWorld(bRec.wo);
							ray = Ray(its.p, wo, ray.time);
							continuePath = pathlength < 32; //TODO rr
						}
						else {
							continuePath = false;
						}
					}
				}//if scene->rayIntersect
				else {
					continuePath = false;
				}
			}//while continuePath
			if (photonStored) i++;
		}//for i = 0 to 100k

		m_kdtree_photons = new PointCloudMap<float, 3>(m_pointCloud_photons);
		if (!placeCaches(scene)) return false;
		fillEnvMapsProgressively();
		Log(EDebug, "Finished. %i Photons and %i Envmaps available.", m_pointCloud_photons.points.size(), m_pointCloud_envmaps.points.size());
		m_pointCloud_photons.points.clear();
		m_photons.clear();
		finalizeEnvmaps(solidAngleMap);


		Log(EDebug, "preprocessing done. try to write an envmap.");
		const Sensor* sensor = scene->getSensor(); //Achtung - was passiert bei >1 sensor in szene?
		RayDifferential sensorRay;
		Vector2i cropSize = sensor->getFilm()->getCropSize();

		/* //uncomment to write 7 x 7 caches to pfm files. Set filename to scene name
		for (int i = 1; i < 8; i++){
			std::vector<float> accMaps;
			int ix = i*cropSize.x / 8;
			for (int j = 1; j < 8; j++){
				int jy = j*cropSize.y / 8;
				Point2 samplePos(ix, jy);
				Point2 apertureSample = Point2(dis(gen), dis(gen));
				float timeSample = dis(gen);
				sensor->sampleRayDifferential(sensorRay, samplePos, apertureSample, timeSample);
				Intersection itss;

				//Erstelle Caches an Schnittpunkt mit Sichtstrahl, falls Oberfläche kein Emitter und keine delta-BSDF
				if (scene->rayIntersect(sensorRay, itss) && !itss.isEmitter() && (its.getBSDF()->getType() & BSDF::ESmooth)){
					Vector3 position(itss.p[0], itss.p[1], itss.p[2]);
					std::vector<std::pair<std::size_t, float>> indices_distances;
					m_kdtree_envmaps->query_knn(1, position, indices_distances); //get k nearest neighbours of photon / cache position
					LocalEnvMap lem = m_envmaps[indices_distances[0].first];
					
					std::vector<float> pdfmap = lem.getEnvmap_pdf();
					for (int l = 0; l < lem.getSize(); l++){
						accMaps.push_back(pdfmap[l]);
					}
				}
				else {
					Log(EDebug, "Ray through pixel doesn't intersect scene. Can't write cache.");
					for (int l = 0; l < 256; l++){
						accMaps.push_back(0.0f);
					}
				}
			}
			MyEnvMap* mem = new MyEnvMap();

			std::stringstream sstm;
			std::string fn = "szene spalte";
			std::string leer = " ";
			std::string pfmstring = ".pfm";
			sstm << fn << ix << leer << pfmstring;

			mem->writeImage(accMaps, 16, 16*7, sstm.str());
			accMaps.clear();
			Log(EDebug, "Writing done for (%i,x)", ix);
		}
		Log(EDebug, "Done with envmaps.");*/

		return true;
	}

	Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const{
		return pathTrace(r, rRec);//Pathtracer der nach envmaps samplet

		//return visualizePhotons(r, rRec); //Gibt Spektrum abh. von nächstem Photon & Entfernung dazu.
	}

	//Platziert die Envmap Caches in der Szene und erstellt schonmal kD-Baum. Die Caches müssen dann noch gefüllt werden.
	bool placeCaches(const Scene* scene){
		if (m_number_of_envmaps > m_photons.size()) {
			Log(EDebug, "Not enough photons (%i in scene) to place %i envmaps", m_photons.size(), m_number_of_envmaps);
			return false;
		}
		else Log(EDebug, "Start placing envmaps. %i Photons in scene.", m_photons.size());

		/*Fill Camera Caches*/
		int alreadyAdded = 0;
		const Sensor* sensor = scene->getSensor(); //Achtung - was passiert bei >1 sensor in szene?
		RayDifferential sensorRay;
		Vector2i cropSize = sensor->getFilm()->getCropSize();
		int borderSize = sensor->getFilm()->getReconstructionFilter()->getBorderSize();
		Log(EDebug, "Border size: %i, cropSize: %i %i", borderSize, cropSize.x, cropSize.y);
		for (int i = 0; i < cropSize.x; i += 8){
			for (int j = 0; j < cropSize.y; j += 8){
				Point2 samplePos(i, j);
				Point2 apertureSample = Point2(dis(gen), dis(gen));
				float timeSample = dis(gen);
				sensor->sampleRayDifferential(sensorRay, samplePos, apertureSample, timeSample);

				Intersection its;

				//Erstelle Caches an Schnittpunkt mit Sichtstrahl, falls Oberfläche kein Emitter und keine delta-BSDF
				if (scene->rayIntersect(sensorRay, its) && !its.isEmitter()){
					const BSDF* bsdf = its.getBSDF();
					if (bsdf->getType() & BSDF::ESmooth){
						Vector3 v(its.p.x, its.p.y, its.p.z);
						Frame f = its.geoFrame;
						bool useLowerHemisphere = bsdf->getType() & BSDF::EBackSide;
						LocalEnvMap lem(useLowerHemisphere, f);
						std::vector<float> envmapcache;
						envmapcache.resize(lem.getSize());

						lem.storeEnvMap_pdf(envmapcache);
						m_envmaps.push_back(lem);
						m_pointCloud_envmaps.points.push_back(v);

						alreadyAdded++;
					}//bsdf is smooth
					else {
						bool continuePath = true;
						Spectrum throughput(1.0f);
						do {
							BSDFSamplingRecord bRec(its, -sensorRay.d, ERadiance);
							const Point2 samplePoint(dis(gen), dis(gen));
							Spectrum bsdfWeight = bsdf->sample(bRec, samplePoint);
							throughput *= bsdfWeight;
							if (throughput.average() > 0.00001f){
								const Vector wo = its.toWorld(bRec.wo);
								sensorRay = Ray(its.p, wo, sensorRay.time);

								if (!scene->rayIntersect(sensorRay, its) || its.isEmitter()){
									continuePath = false;
								}
								else if (its.getBSDF()->getType() & BSDF::ESmooth){
									bsdf = its.getBSDF();

									continuePath = false;
									Vector3 v(its.p.x, its.p.y, its.p.z);
									Frame f = its.geoFrame;
									bool useLowerHemisphere = bsdf->getType() & BSDF::EBackSide;
									LocalEnvMap lem(useLowerHemisphere, f);
									std::vector<float> envmapcache;
									envmapcache.resize(lem.getSize());

									lem.storeEnvMap_pdf(envmapcache);
									m_envmaps.push_back(lem);
									m_pointCloud_envmaps.points.push_back(v);

									alreadyAdded++;
								}//if scene->intersect && bsdf smooth
							}//if throughput > 0.00001
							else {
								continuePath = false;
							}//if throughput <= 0.00001
						} while (continuePath);
						//TODO Verfolgen bis zu erstem smoothen schnittpunkt
					}//bsdf not smooth
				}//scene->rayIntersect && !its.isEmitter
			}
		}
		alreadyAdded = m_pointCloud_envmaps.points.size();
		Log(EDebug, "Added %i caches for camera rays", alreadyAdded);

		/*Add Photon Caches*/
		if (m_number_of_envmaps - alreadyAdded < m_photons.size()){
			for (int i = alreadyAdded; i < m_number_of_envmaps; i++){ //einfach mal die ersten # photonen
				Photon photon = m_photons[i];
				Vector3 v(photon.position.x, photon.position.y, photon.position.z);
				Frame f = photon.surface_normal_frame;
				bool useLowerHemisphere = photon.bsdf->getType() & BSDF::EBackSide;
				LocalEnvMap lem(useLowerHemisphere, f);
				int lem_size = lem.getSize();
				std::vector<float> envmapcache;
				envmapcache.resize(lem_size);

				lem.storeEnvMap_pdf(envmapcache);
				m_envmaps.push_back(lem);
				m_pointCloud_envmaps.points.push_back(v);
			}//for number_of_envmaps

		}
		else Log(EDebug, "Only %i photons in scene for %i envmaps. abort.", m_photons.size(), m_number_of_envmaps);
		m_envmaps_available = m_envmaps.size() > 0;

		Log(EDebug, "Done placing %i of %i envmapcaches.", m_envmaps.size(), m_number_of_envmaps);

		return m_envmaps_available;
	}//place caches

	void fillEnvMapsProgressively(){
		Log(EDebug, "Start filling envmaps");
		int k = m_samples_for_envmap;
		int actuallyAddedMaps = 0;
		//erstmal nur an Photonen-Stellen, ich weiß grade nicht wo die Kamera ist.
		int alreadyAdded = 0;
		for (int i = 0; i < m_envmaps.size(); i++){
			LocalEnvMap lem = m_envmaps[i];
			std::vector<float> envmapcache = lem.getEnvmap_pdf();
			Vector3 v = m_pointCloud_envmaps.points[i];
			std::vector<std::pair<std::size_t, float>> indices_distances;
			m_kdtree_photons->query_knn(k, v, indices_distances); //get k nearest neighbours of photon / cache position
			if (indices_distances.size() == k){
				double lem_width = lem.getWidth();
				double lem_height = lem.getHeight();
				bool useLowerHemisphere = lem.doesUseBothHemispheres();
				double sum = 0;

				for (int j = 0; j < k; j++){
					Photon photon_j = m_photons[indices_distances[j].first];

					Vector3 dir = lem.toLocal(photon_j.w_i);
					float cos = lem.cosTheta(dir);
					if (cos >= 0 || useLowerHemisphere){//TODO wieso !bsdfIsSmooth?! nicht eher useLowerHemisphere?
						dir = Vector3(dir.x, dir.z, dir.y);
						Point2d texCoord = lem.directionToTexture(dir);
						int x = (int)floor(texCoord.x * lem_width);
						int y = (int)floor(texCoord.y * lem_height);
						double photonenergy = (double)photon_j.energy_avg;
						//if (indices_distances[j].second <= 0.1) indices_distances[j].second = 0.1;//Avoid dividing by zero or too small numbers //sonst ist der Eintrag in der Envmap an einer stelle total rieseig (0.99) und sonst seeehr nah an 0
						//photonenergy /= indices_distances[j].second;

						float a = texCoord.x * lem_width - x;
						float b = texCoord.y * lem_height - y;
						assert(0.f <= a && a < 1);
						assert(0.f <= b && b < 1);

						int x_next = x;
						int y_next = y;

						if (a < 0.5f){
							x_next = std::max(x - 1, 0);
							a = 0.5f - a;
						}
						else if (a == 0.5f){
							x_next = x;
							a = 0;
						}
						else {
							x_next = std::min(x + 1, lem.getWidth() - 1);
							a -= 0.5f;
						}
						if (b < 0.5f){
							y_next = std::max(0, y - 1);
							b = 0.5f - b;
						}
						else if (b == 0.5f){
							y_next = y;
							b = 0;
						}
						else {
							y_next = std::min(lem.getHeight() - 1, y + 1);
							b -= 0.5f;
						}

						float w1 = (1 - a)*(1 - b);
						float w2 = (1 - a)*b;
						float w3 = a*(1 - b);
						float w4 = a*b;

						int i1 = x + y*lem_width;
						int i2 = x + y_next*lem_width;
						int i3 = x_next + y*lem_width;
						int i4 = x_next + y_next*lem_width;

						envmapcache[i1] += photonenergy*w1;
						envmapcache[i2] += photonenergy*w2;
						envmapcache[i3] += photonenergy*w3;
						envmapcache[i4] += photonenergy*w4;

//						envmapcache[x + y*lem_width] += photonenergy; //TODO gewichten mit distance?
					}
					else {
						//Log(EDebug, "Don't use Photon %i. cos: %f", j, cos);
					}
				}//for k nearest photons
				if (i % 10000 == 0){
					Log(EDebug, "Envmap %i after progressively filling", i);
					for (int a = 0; a < lem.getHeight(); a++){
						if (lem.getWidth() >= 16) Log(EDebug, "Values: %f %f %f %f %f %f %f %f | %f %f %f %f %f %f %f %f", envmapcache[a * lem_width + 0], envmapcache[a * lem_width + 1], envmapcache[a * lem_width + 2], envmapcache[a * lem_width + 3], envmapcache[a * lem_width + 4], envmapcache[a * lem_width + 5], envmapcache[a * lem_width + 6], envmapcache[a * lem_width + 7], envmapcache[a * lem_width + 8], envmapcache[a * lem_width + 9], envmapcache[a * lem_width + 10], envmapcache[a * lem_width + 11], envmapcache[a * lem_width + 12], envmapcache[a * lem_width + 13], envmapcache[a * lem_width + 14], envmapcache[a * lem_width + 15]);
						else Log(EDebug, "Values: %f %f %f %f %f %f %f %f", envmapcache[a * lem_width + 0], envmapcache[a * lem_width + 1], envmapcache[a * lem_width + 2], envmapcache[a * lem_width + 3], envmapcache[a * lem_width + 4], envmapcache[a * lem_width + 5], envmapcache[a * lem_width + 6], envmapcache[a * lem_width + 7]);
					}
				}

				lem.storeEnvMap_pdf(envmapcache);
				m_envmaps[i] = lem;
			}//if indices_distances.size == k
		}//for all envmaps

		Log(EDebug, "Done filling envmaps with %i additional samples.", k);
	}//fill envmaps progressively


	//Normiert auf 1 und verrechnet dann zu solid angle pdf
	void finalizeEnvmaps(const std::vector<float> solidAngleMap){
		Log(EDebug, "Start finalizing envmaps. %i available.", m_envmaps.size());
		assert(m_envmaps.size() == m_pointCloud_envmaps.points.size());
		int erased = 0;
		int j = 0;
		const std::vector<float> solidAngleBothHemispheres = createSolidAngleBothHemispheres(solidAngleMap);
		for (int i = 0; i < m_envmaps.size(); i++){
			LocalEnvMap lem = m_envmaps[i];
			std::vector<float> envmapcache = lem.getEnvmap_pdf();
			double sum = 0, newsum = 0, newersum = 0;//debug
			int lem_width = lem.getWidth();
			int lem_size = lem.getSize();

			/*Gaussfilter*/
			float a = 0.25f;
			int lem_height = lem.getHeight();
			std::vector<float> gaussfiltered = envmapcache;
			float b = 0.125f;
			float c = 0.0625f;
			for (int x = 0; x < lem_width; x++){
				for (int y = 0; y < lem_height; y++){
					int x1 = (x + lem_width - 1) % lem_width;
					int x2 = (x + 1) % lem_width;
					int y0 = y*lem_width;
					int y1 = ((y + 1) % lem_height)*lem_width;
					int y2 = ((y - 1 + lem_height) % lem_height)*lem_width;
					float newval = a * envmapcache[x + y0]
						+ b* envmapcache[x1 + y0]
						+ b* envmapcache[x2 + y0]
						+ b* envmapcache[x + y1]
						+ b* envmapcache[x + y2]
						+ c* envmapcache[x1 + y1]
						+ c* envmapcache[x1 + y2]
						+ c* envmapcache[x2 + y1]
						+ c* envmapcache[x2 + y2];
					gaussfiltered[x + y*lem_width] = newval;
				}
			}
			envmapcache = gaussfiltered;

			double min = 1000;
			for (int j = 0; j < lem_size; j++){
				sum += envmapcache[j];
				//if (envmapcache[j] > 0 && envmapcache[j] < min)min = envmapcache[j];
			}
			if (sum <= 0){
				m_envmaps.erase(m_envmaps.begin() + i);
				m_pointCloud_envmaps.points.erase(m_pointCloud_envmaps.points.begin() + i);
				i--;
				erased++;
			}
			else {
				double avg_half = sum / (2 * lem_size);
				if (i == 10000){
					Log(EDebug, "Envmap %i before finalizing, sum: %f, min: %f,", i, sum, min);
					for (int a = 0; a < lem.getHeight(); a++){
						if (lem.getWidth() >= 16) Log(EDebug, "Values: %f %f %f %f %f %f %f %f | %f %f %f %f %f %f %f %f", envmapcache[a * lem_width + 0], envmapcache[a * lem_width + 1], envmapcache[a * lem_width + 2], envmapcache[a * lem_width + 3], envmapcache[a * lem_width + 4], envmapcache[a * lem_width + 5], envmapcache[a * lem_width + 6], envmapcache[a * lem_width + 7], envmapcache[a * lem_width + 8], envmapcache[a * lem_width + 9], envmapcache[a * lem_width + 10], envmapcache[a * lem_width + 11], envmapcache[a * lem_width + 12], envmapcache[a * lem_width + 13], envmapcache[a * lem_width + 14], envmapcache[a * lem_width + 15]);
						else Log(EDebug, "Values: %f %f %f %f %f %f %f %f", envmapcache[a * lem_width + 0], envmapcache[a * lem_width + 1], envmapcache[a * lem_width + 2], envmapcache[a * lem_width + 3], envmapcache[a * lem_width + 4], envmapcache[a * lem_width + 5], envmapcache[a * lem_width + 6], envmapcache[a * lem_width + 7]);
					}
				}
				//int bigger = 0, smaller = 0, zero = 0;
				//for (int j = 0; j < lem_size; j++){
				//	if (envmapcache[j] == 0) zero++;
				//	else if (envmapcache[j] < avg_half) smaller++;
				//	else bigger++;
				//}
				//if (i >= 10000 && i < 10005) Log(EDebug, "avg/2: %f. bigger: %i, smaller: %i, 0: %i", avg_half, bigger, smaller, zero);

				/*sum = 0;
				for (int j = 0; j < lem_size; j++){
				if (envmapcache[j] < avg_half) envmapcache[j] = avg_half;
				sum += envmapcache[j];
				}*/

				//for (int j = 0; j < lem_size; j++){
				//	if (envmapcache[j] < min) {
				//		envmapcache[j] = min;
				//		sum += min;
				//	}
				//}
				//TODO: erst normalisieren, dann minimum, dann 0.x
				for (int j = 0; j < lem_size; j++){
					envmapcache[j] /= sum;
					if (envmapcache[j] < 0.0002) {
						envmapcache[j] = 0.0002;
					}
					newsum += envmapcache[j];
				}

				for (int j = 0; j < lem_size; j++){
					envmapcache[j] /= newsum;
					assert(envmapcache[j] > 0.0f);
					newersum += envmapcache[j];
				}

				if (i == 10000){
					Log(EDebug, "Envmap %i after finalizing, newsum: %f, sum: %f, newersum: %f", i, newsum, sum, newersum);
					for (int a = 0; a < lem.getHeight(); a++){
						if (lem.getWidth() >= 16) Log(EDebug, "Values: %f %f %f %f %f %f %f %f | %f %f %f %f %f %f %f %f", envmapcache[a * lem_width + 0], envmapcache[a * lem_width + 1], envmapcache[a * lem_width + 2], envmapcache[a * lem_width + 3], envmapcache[a * lem_width + 4], envmapcache[a * lem_width + 5], envmapcache[a * lem_width + 6], envmapcache[a * lem_width + 7], envmapcache[a * lem_width + 8], envmapcache[a * lem_width + 9], envmapcache[a * lem_width + 10], envmapcache[a * lem_width + 11], envmapcache[a * lem_width + 12], envmapcache[a * lem_width + 13], envmapcache[a * lem_width + 14], envmapcache[a * lem_width + 15]);
						else Log(EDebug, "Values: %f %f %f %f %f %f %f %f", envmapcache[a * lem_width + 0], envmapcache[a * lem_width + 1], envmapcache[a * lem_width + 2], envmapcache[a * lem_width + 3], envmapcache[a * lem_width + 4], envmapcache[a * lem_width + 5], envmapcache[a * lem_width + 6], envmapcache[a * lem_width + 7]);
					}
				}//debug for i=10k
				if (!lem.doesUseBothHemispheres()){
					lem.setEnvMap(envmapcache, solidAngleMap);
				}
				else{
					lem.setEnvMap(envmapcache, solidAngleBothHemispheres);
				}
				m_envmaps[i] = lem;

				envmapcache = m_envmaps[i].getEnvmap_pdf();
				if (i == 10000){
					Log(EDebug, "m_envmaps[i].getEnvmap_pdf %i after adapting to solid angle, newsum: %f, sum: %f, newersum: %f", i, newsum, sum, newersum);
					for (int a = 0; a < lem.getHeight(); a++){
						if (lem.getWidth() >= 16) Log(EDebug, "Values: %f %f %f %f %f %f %f %f | %f %f %f %f %f %f %f %f", envmapcache[a * lem_width + 0], envmapcache[a * lem_width + 1], envmapcache[a * lem_width + 2], envmapcache[a * lem_width + 3], envmapcache[a * lem_width + 4], envmapcache[a * lem_width + 5], envmapcache[a * lem_width + 6], envmapcache[a * lem_width + 7], envmapcache[a * lem_width + 8], envmapcache[a * lem_width + 9], envmapcache[a * lem_width + 10], envmapcache[a * lem_width + 11], envmapcache[a * lem_width + 12], envmapcache[a * lem_width + 13], envmapcache[a * lem_width + 14], envmapcache[a * lem_width + 15]);
						else Log(EDebug, "Values: %f %f %f %f %f %f %f %f", envmapcache[a * lem_width + 0], envmapcache[a * lem_width + 1], envmapcache[a * lem_width + 2], envmapcache[a * lem_width + 3], envmapcache[a * lem_width + 4], envmapcache[a * lem_width + 5], envmapcache[a * lem_width + 6], envmapcache[a * lem_width + 7]);
					}
				}//debug for i=10k
			}
		}
		Log(EDebug, "Finalized envmaps. Erased %i, %i left. Build kD-Tree.", erased, m_envmaps.size());
		assert(m_envmaps.size() == m_pointCloud_envmaps.points.size());
		m_envmaps_available = m_envmaps.size() > 0;
		if (m_envmaps_available) {
			m_kdtree_envmaps = new PointCloudMap<float, 3>(m_pointCloud_envmaps);
			Log(EDebug, "Done building kd-Tree for %i envmaps", m_pointCloud_envmaps.points.size());
		}
	}//finalize envmaps

private:

	std::vector<float> createSolidAngleMap(){
		std::vector<float> solidAngle;
		solidAngle.resize(LocalEnvMap::size_oneHemisphere);
		const uint32_t numSamples = 1000 * LocalEnvMap::size_oneHemisphere;
		for (uint32_t i = 0; i < numSamples; i++){
			const float xi_1 = dis(gen);
			const float xi_2 = dis(gen);
			const float theta = acosf(1.0f - xi_1);
			const float phi = M_PI_FLT*2.0f*xi_2;

			assert(theta >= 0.0f);
			assert(theta <= 0.5f*M_PI_FLT);

			float st = std::sin(theta);
			Vector3f wo(-st*std::cos(phi), std::cos(theta), st * std::sin(phi));
			assert(wo[1] >= 0.0f);

			Vector3 p_ = wo / (std::abs(wo[0]) + std::abs(wo[1]) + std::abs(wo[2]));
			Point2d p;
			p.y = p_[0] + p_[2];
			p.x = p_[0] - p_[2] - 1;
			p.x += 2;
			p.x *= 0.5f;
			p.y += 1;
			p.y *= 0.5f;
			if (p.x < 1 || p.y < 1){
				solidAngle[(int)(LocalEnvMap::width_oneHemisphere*p.x) + LocalEnvMap::width_oneHemisphere*(int)(LocalEnvMap::height_oneHemisphere*p.y)] += 1;
			}
		}
		for (int i = 0; i < solidAngle.size(); i++){
			if (solidAngle[i] <= 0) solidAngle[i] = 1;
			solidAngle[i] *= M_PI_FLT*2.0f / numSamples;
			assert(solidAngle[i] > 0.0f);
		}
		for (int i = 0; i < 16; i++){
			Log(EDebug, "Solid angles: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", solidAngle[i * 16 + 0], solidAngle[i * 16 + 1], solidAngle[i * 16 + 2],
				solidAngle[i * 16 + 3], solidAngle[i * 16 + 4], solidAngle[i * 16 + 5], solidAngle[i * 16 + 6], solidAngle[i * 16 + 7], solidAngle[i * 16 + 8],
				solidAngle[i * 16 + 9], solidAngle[i * 16 + 10], solidAngle[i * 16 + 11], solidAngle[i * 16 + 12], solidAngle[i * 16 + 13],
				solidAngle[i * 16 + 14], solidAngle[i * 16 + 15]);
		}
		MyEnvMap* mem = new MyEnvMap();
		return solidAngle;
	}

	std::vector<float> createSolidAngleBothHemispheres(const std::vector<float> solidAngleOneHemisphere){
		std::vector<float> solidAngle;
		solidAngle.resize(2 * LocalEnvMap::size_oneHemisphere);

		for (int j = 0; j < LocalEnvMap::height_oneHemisphere; j++){
			for (int i = 0; i < LocalEnvMap::width_oneHemisphere; i++){
				solidAngle[j*LocalEnvMap::width_oneHemisphere + i] = solidAngleOneHemisphere[j*LocalEnvMap::width_oneHemisphere + i];
				solidAngle[j*LocalEnvMap::width_oneHemisphere + LocalEnvMap::width_oneHemisphere + i] = solidAngleOneHemisphere[j*LocalEnvMap::width_oneHemisphere + i];
			}
		}
		return solidAngle;
	}

	Spectrum visualizePhotons(const RayDifferential &r, RadianceQueryRecord &rRec) const{
		RayDifferential ray(r);
		Intersection its;
		if (!rRec.scene->rayIntersect(ray, its)){
			return Spectrum(0.0f);
		}

		std::vector<std::pair<std::size_t, float>> indices_distances;
		Vector3 v(its.p[0], its.p[1], its.p[2]);
		m_kdtree_photons->query_knn(1, v, indices_distances);

		if (indices_distances.empty()) return Spectrum(0.f);

		Photon closest_photon = m_photons[indices_distances[0].first];
		float dist = indices_distances[0].second;
		if (dist == 0) return Spectrum(1.0f);

		const BSDF* bsdf = its.getBSDF();
		BSDFSamplingRecord bRec(its, its.toLocal(-r.d));
		bRec.wi = its.toLocal(closest_photon.w_i); //Ich bin nicht ganz sicher, was bRec(its, ...) macht, deshalb nochmal hier.
		bRec.wo = its.toLocal(-r.d); //... TODO ist wi und wo richtigrum oder vertauscht?
		Spectrum bsdfval = bsdf->eval(bRec);

		return bsdfval * Spectrum(closest_photon.energy_avg) / dist; //naja damit man halt irgendwas sieht
	}

	Spectrum pathTrace(const RayDifferential &r, RadianceQueryRecord &rRec) const{
		if (m_maxDepth <= 0) return Spectrum(0.0f);
		Spectrum Li(0.0f);
		const Scene* scene = rRec.scene;

		Spectrum f_X(1.0f); //Sensor Sensitivity erstmal weglassen

		RayDifferential ray(r);
		bool intersectionExists = rRec.rayIntersect(ray);
		if (!intersectionExists){
			if (m_use_own_envmap){
				Li += f_X*m_envmap->getTextureAtDirection(ray.d);
				//return m_envmap->checkIfConversionWorksRect(ray.d,false);
			}
			else {
				const Emitter *environment = scene->getEnvironmentEmitter();
				if (environment){
					Spectrum value = environment->evalEnvironment(ray);
					Li += f_X*value;
				}
			}
			return Li;
		}

		Intersection &its = rRec.its;
		bool pathTerminated = false;
		int i = 0;
		float w_I = 0;
		float w_E = 0;
		if (m_use_fix_w_I){
			w_I = m_fix_w_I;
		}
		if (m_use_fix_w_E){
			w_E = m_fix_w_E;
		}

		while (!pathTerminated){
			i++;

			/* Check implicit path */
			if (its.isEmitter()){
				if (i > 1){
					//w_I wurde schon in der while-Schleife davor berechnet, weil da die Objekte dafür direkt da waren.
				}
				else {
					w_I = 1;
				}
				Spectrum l = its.Le(-ray.d);
				//Log(EDebug, "Done. Li: (%f,%f,%f), f_X: (%f,%f,%f), w: %f", l[0], l[1], l[2], f_X[0], f_X[1], f_X[2], w_I);
				Li += w_I*f_X*l;
				pathTerminated = true;
			}
			else {

				//bool sampleBSDF = true; //TODO das dynamisch machen. Mal nach bsdf samplen, mal nach envmap, vllt mal nur nach cosinus

				const BSDF *bsdf = its.getBSDF(ray); //BSDF bzgl w_o bzw einfallendem Pfad bzw später ausgehendem licht
				bool bsdfSmooth = bsdf->getType() & BSDF::ESmooth;
				DirectSamplingRecord dRec(its);

				//initialize stuff for if clauses
				bool envmap_useful = false;
				LocalEnvMap lem = m_envmaps[0];
				double bestCos = 0;
				/*Get best nearest envmapcache*/
				if (!m_sample_BSDF_only){
					Vector3 position(its.p[0], its.p[1], its.p[2]);
					std::vector<std::pair<std::size_t, float>> indices_distances;
					m_kdtree_envmaps->query_knn(1, position, indices_distances); //get k nearest neighbours of photon / cache position
					
					if (indices_distances.size() == 1){ //sollte nicht so weit weg sein und ähnliche normale haben. Für 5 nächste Caches auf 5 ändern.
						lem = m_envmaps[indices_distances[0].first];
						Vector envmapNormal = lem.getNormal();
						Vector itsNormal = its.geoFrame.n;
						bestCos = itsNormal[0] * envmapNormal[0] + itsNormal[1] * envmapNormal[1] + itsNormal[2] * envmapNormal[2];
						//for (int i = 1; i < 1; i++){//für die nächst-nahen Caches
						//	LocalEnvMap nextLem = m_envmaps[indices_distances[i].first];
						//	envmapNormal = nextLem.getNormal();
						//	float nextCos = itsNormal[0] * envmapNormal[0] + itsNormal[1] * envmapNormal[1] + itsNormal[2] * envmapNormal[2];
						//	if (nextCos > bestCos || (nextCos == bestCos && rRec.nextSample1D() > 0.6)){ //Wenn gleich: Nimm zufällig vllt doch andere
						//		bestCos = nextCos;
						//		lem = nextLem;
						//	}
						//}
						envmap_useful = bestCos > 0.9;
					}//if indices_distances richtige Größe
					else {
						Log(EDebug, "inidces distances != 0 this should not happen. return Li.");
						return Li; //jaja das ist suboptimal, aber erstmal sicher, falls knn query fehlschlägt
					}
				}

				/* Make explicit connection */
				if (bsdfSmooth){ //Nur bei nicht-Delta-BSDFs!
					Spectrum value = scene->sampleEmitterDirect(dRec, rRec.nextSample2D());
					if (!value.isZero()){
						BSDFSamplingRecord bRec(its, its.toLocal(dRec.d)); //bRec wird direkt mit den Daten gefüllt, das Sample wurde mit dRec erstellt
						const Spectrum bsdfVal = bsdf->eval(bRec); //Evaluate BSDF*cos(theta)  //theta bzgl w_o
						if (!bsdfVal.isZero()){ //und Geometriebedingungen, hier weggelassen
							float samplingPdf = 0;

							if (!m_sample_BSDF_only && envmap_useful){
								samplingPdf = lem.getProbabilityForEnvmapDirection(its.toLocal(dRec.d));
							}
							else{
								samplingPdf = bsdf->pdf(bRec); //pdf über solid angle //Wie wahrscheinlich wäre es gewesen, dieses Sample über die normale pdf für Strahlfortsetzen zu erhalten?
								//Log(EDebug, "Explicit connection: Sample BSDF");
							}
							Float lumPdf = scene->pdfEmitterDirect(dRec); //Wie wahrscheinlich wäre es gewesen, dieses Sample zu samplen, hätte man ein Emittersample holen wollen?

							//TODO: if (dRec.Measure == EArea) dann muss man noch was machen. Kucken ob das passiert.
							if (lumPdf == 0 && samplingPdf == 0) w_E = 0; //Hm. lumPdf sollte eigentlich nicht 0 sein. samplingPdf kann bei den Envmaps aktuell (29.10.) schon mal 0 werden.
							else if (!m_use_fix_w_E) w_E = lumPdf*lumPdf / (lumPdf*lumPdf + samplingPdf*samplingPdf);//weight entspricht wahrscheinlich w_E mit beta=2 
							Li += w_E*f_X*bsdfVal*value; //evtl fehlen Geometriefaktoren für projected solid angle. grade alles mit normalen solid angles.
						}
					}//value on emitter != zero
				}//bsdf smooth


				/* Russian Roulette */
				if (i >= m_rrDepth){
					Spectrum diffRef = bsdf->getDiffuseReflectance(its);
					Spectrum specRef = bsdf->getSpecularReflectance(its);
					float alpha = diffRef.average() + specRef.average();
					if (alpha > 1) alpha = 1;
					float random = rRec.nextSample1D();
					if (random >= alpha){//>= statt größer, weil der assert(alpha > 0) auch schon mal fehlgeschlagen hat.
						pathTerminated = true;
						return Li;
					}
					else {
						assert(alpha > 0);
						f_X /= alpha;
					}
				}

				/* Extend Path */
				float samplingPdf = -1.0f;

				if (m_sample_BSDF_only || !bsdfSmooth || !m_envmaps_available || !envmap_useful){ //wenn BSDF nicht smooth, ist da eh kein Photon gespeichert.
					if (!sampleBSDF(ray, rRec, bsdf, samplingPdf, its, f_X)) return Li;
				}
				else { //sample envmapcache
					float roughness = bsdf->getRoughness(its, 0);
					float local_mis_alpha = std::min(roughness, 0.9f);

					if (!m_sample_cache_only && rRec.nextSample1D() > local_mis_alpha){
						if (!sampleBSDF(ray, rRec, bsdf, samplingPdf, its, f_X)) return Li;

						Vector sampleDirEnvmap;
						sampleDirEnvmap = lem.toLocal(ray.d); //TODO Achtung: Falls sehr flach gesampelt wurde und der cosinus zur envmap nicht 1 ist, kann es passieren, dass aus envmapsicht nach unten gesampelt wurde?
						if (sampleDirEnvmap.z <= 0) {
							Log(EDebug, "Sample dir envmap local: (%f,%f,%f)", sampleDirEnvmap.x, sampleDirEnvmap.y, sampleDirEnvmap.z);
							sampleDirEnvmap = its.toLocal(ray.d);

							Log(EDebug, "Sample dir its local: (%f,%f,%f)", sampleDirEnvmap.x, sampleDirEnvmap.y, sampleDirEnvmap.z);
						}// else Log(EDebug, "Sample dir local: (%f,%f,%f)", sampleDirEnvmap.x, sampleDirEnvmap.y, sampleDirEnvmap.z);
						assert(!(sampleDirEnvmap.x == 0 && sampleDirEnvmap.y == 0 && sampleDirEnvmap.z == 0));
						if (sampleDirEnvmap.z <= 0) {
							Log(EWarn, "This should not happen. sampleDirEnvmap: (%f,%f,%f)", sampleDirEnvmap.x, sampleDirEnvmap.y, sampleDirEnvmap.z);
							if (sampleDirEnvmap.z > -0.000001) {
								sampleDirEnvmap.z = 0;
								Log(EWarn, "Set sampleDirEnvmap to 0.");
							}
						}
						assert(sampleDirEnvmap.z >= 0);

						float envmapPdf = lem.getProbabilityForEnvmapDirection(sampleDirEnvmap); //nimmt mitsuba-koordinaten und konvertiert selbst
						assert(local_mis_alpha < 1);
						float localMisWeight = samplingPdf / ((1 - local_mis_alpha) * (samplingPdf + envmapPdf));
						f_X *= localMisWeight;
					}
					else {

						//get random texel from envmap and pdf
						Point2d texel = lem.sampleLocalEnvMapTexel(samplingPdf, rRec.nextSample1D(), rRec.nextSample2D());
						//convert to direction
						Vector sampleDir = lem.textureToDirection(texel);
						sampleDir = Vector(sampleDir.x, sampleDir.z, sampleDir.y);//Mitsuba- und Octahedron-Koordinaten haben y- und z-Achse getauscht

						if (bestCos < 0.99){//envmap zeigt nicht in genau selbe Richtung - Verschiedene Frames abfangen
							//Log(EDebug, "best found cos: %f. transform direction to its coordinates.", bestCos);
							sampleDir = its.toLocal(lem.localDirToGlobalDir(sampleDir)); //Lokale envmap-Richtung -> Welt-Richtung -> lokale its-Richtung
						}
						assert(sampleDir.length() > 0.0f);
						sampleDir /= sampleDir.length();

						//eval bsdf
						const Vector wo = sampleDir; //w_o: Richtung, in die Sichtstrahl weiterverfolgt wird = Richtung, aus der später Licht _einfällt_
						const Vector wi = its.toLocal(-ray.d); //w_i: Richtung, aus der Sichtstrahl kommt
						BSDFSamplingRecord bRec(its, wi, wo); //schon ERadiance, oder? //Achtung! Cos(theta) wird für wo berechnet.
						Spectrum bsdfValue = bsdf->eval(bRec);
						if (samplingPdf == 0) Log(EDebug, "Envmap: samplingPdf = 0! This should not happen.");
						//multiply throughput
						assert(samplingPdf > 0.f);
						//assert(samplingPdf > 0.f);
						f_X *= bsdfValue / samplingPdf;

						assert(local_mis_alpha > 0);
						if (!m_sample_cache_only){
							float bsdfPdf = bsdf->pdf(bRec);
							float localMisWeight = samplingPdf / (local_mis_alpha * (samplingPdf + bsdfPdf));
							f_X *= localMisWeight;
						}
						ray = Ray(its.p, its.toWorld(sampleDir), ray.time); //its.toWorld nicht ganz korrekt, evtl ist oberflächennormale der envmap bisschen anders als an its
					}
				}
				if (scene->rayIntersect(ray, its)){ //TODO: rRec.intersectionExists(ray) besser? Und dann über rRec.its its neu setzen
					/* w_I schon hier setzen, im nächsten while-Durchlauf ist dRec weg.*/
					if (its.isEmitter()){
						dRec.setQuery(ray, its);
						Float lumPdf = scene->pdfEmitterDirect(dRec); //Wie wahrscheinlich wäre es gewesen, dieses Sample zu samplen, hätte man ein Emittersample holen wollen?
						assert(samplingPdf > 0.0f);
						if (lumPdf <= 0.0f){
							//Log(EDebug, "lumPdf = %f for emitter. SamplingPdf: %f return Spectrum(1). Path length: %i", lumPdf, samplingPdf, i);
							//TODO warum passiert das so oft?
							//return Spectrum(1.0f);
						}
						//assert(lumPdf > 0.0f);
						//TODO: if (dRec.Measure == EArea) dann muss man noch was machen. Kucken ob das passiert.
						if (!m_use_fix_w_I){
							if (bsdfSmooth)	w_I = samplingPdf*samplingPdf / (samplingPdf*samplingPdf + lumPdf*lumPdf); //Power heuristic beta=2
							else w_I = 1;
						}
					}

					/*
					bsdf->sample()
					pdf
					Will record the probability with respect to solid angles (or the discrete probability when a delta component is sampled)
					Returns:
					- bei sample( ohne pdf)
					The BSDF value divided by the probability density of the sample sample
					(multiplied by the cosine foreshortening factor when a non-delta component is sampled)
					A zero spectrum means that sampling failed.
					- bei sample(mit pdf)
					The BSDF value (multiplied by the cosine foreshortening factor when a non-delta component is sampled).
					A zero spectrum means that sampling failed.


					scene->pdfEmitterDirect(dRec):
					Returns
					The density expressed with respect to the requested measure (usually ESolidAngle)

					*/
				} //if(scene->rayIntersect(ray,its))
				else {
					/*Read environment map*/
					if (m_use_own_envmap){
						//return m_envmap->checkIfConversionWorksRect(ray.d, false);
						Spectrum value = m_envmap->getTextureAtDirection(ray.d);
						//float lumPdf = 1 / (4 * M_PI_FLT); //TODO Wieso braucht man hier nochmal Gewichte? ist lumPdf nicht eh konstant?
						//if (!use_fix_w_I) w_I = bsdfPdf*bsdfPdf / (bsdfPdf*bsdfPdf + lumPdf*lumPdf);
						w_I = 1; //TODO
						Li += w_I*f_X*value;//ich nehm einfach mal gleichmäßige wahrscheinlichkeit an
					}
					else {
						const Emitter *environment = scene->getEnvironmentEmitter();
						if (environment){
							Spectrum value = environment->evalEnvironment(ray);
							if (environment->fillDirectSamplingRecord(dRec, ray)){
								dRec.measure = EArea;
								const Float lumPdf = scene->pdfEmitterDirect(dRec);
								assert(lumPdf > 0.0f && samplingPdf > 0.0f);
								if (!m_use_fix_w_I) w_I = samplingPdf*samplingPdf / (samplingPdf*samplingPdf + lumPdf*lumPdf);
								Li += w_I*f_X*value;
							}
						}
					}
					pathTerminated = true; //hm eig braucht man nichts mehr setzen
					return Li;
				}
			}
			if (i >= m_maxDepth) pathTerminated = true;
		} //while
		return Li;
	}

	bool sampleBSDF(Ray& ray, RadianceQueryRecord& rRec, const BSDF* bsdf, float& samplingPdf, Intersection& its, Spectrum& f_X) const {
		//Log(EDebug, "extend path: Sample bsdf.");
		BSDFSamplingRecord bRec(its, rRec.sampler, ERadiance);
		Intersection previousIts = its;
		Point previousPoint = its.p;
		Spectrum bsdfWeight = bsdf->sample(bRec, samplingPdf, rRec.nextSample2D());
		if (bsdfWeight.isZero()){ //bsdf->sample == zero  =>  Sampling fehlgeschlagen
			//Log(EDebug, "bsdfWeight is zero. sampling didn't work.");
			f_X = Spectrum(0.0f);
			return false;
			//return Li; //Wow. anscheinend passiert das doch öfter mal. Hatte davor nur Spectrum(0) zurückgegeben deswegen bei Veach so komische Schatten hinter den Brettern.
		}
		//Log(EDebug, "bsdf sampling pdf: %f", samplingPdf);
		f_X *= bsdfWeight; //TOOD bsdfWeight müsste eig schon durch pdf geteilt sein. Also doch kein TODO. (?)
		const Vector wo = its.toWorld(bRec.wo);
		ray = Ray(its.p, wo, ray.time);
		return true;
	}


	MTS_DECLARE_CLASS()
protected:
	bool m_show_caches;

	int m_traced_photon_paths;
	int m_number_of_envmaps;
	std::vector<Photon> m_photons;
	PointCloud<float, 3> m_pointCloud_photons;
	PointCloudMap<float, 3>* m_kdtree_photons;
	std::vector<LocalEnvMap> m_envmaps;
	PointCloud<float, 3> m_pointCloud_envmaps;
	PointCloudMap<float, 3>* m_kdtree_envmaps;

	float m_fix_w_I; //fix MIS weight for implicipt paths; -1 => compute w_I
	float m_fix_w_E; //fix MIS weight for explicit paths; -1 => compute w_E
	bool m_use_fix_w_I;
	bool m_use_fix_w_E;
	bool m_use_own_envmap;
	bool m_sample_BSDF_only;
	bool m_sample_cache_only;
	MyEnvMap *m_envmap;

	int m_samples_for_envmap;

	bool m_envmaps_available;

	std::uniform_real_distribution<double> dis;
	std::mt19937 gen;
};
MTS_IMPLEMENT_CLASS_S(MyPathTracerAndPhotonMapper, false, MonteCarloIntegrator)
MTS_EXPORT_PLUGIN(MyPathTracerAndPhotonMapper, "My Path Tracer and Photon Mapper");
MTS_NAMESPACE_END