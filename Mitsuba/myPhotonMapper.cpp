#include <mitsuba/render/scene.h>
#include <mitsuba/nnpointcloud.h>
#include <mitsuba/nnkdtree.h>
#include <mitsuba/myenvmap.h>
#include <ctime>
#include <array>

MTS_NAMESPACE_BEGIN

struct Photon{
	Point position;
	Frame surface_normal_frame; //Jaja normale braucht weniger platz. sp‰ter.
	Spectrum energy;
	const BSDF* bsdf;
	Vector w_i; //im Moment in Weltkoordinaten

	Photon(Point position, Frame normal, Spectrum energy, const BSDF* bsdf, Vector w_i){
		this->position = position;
		this->surface_normal_frame = normal;
		this->energy = energy;
		this->bsdf = bsdf;
		this->w_i = w_i;
	}
};


class MyPhotonMapper : public	MonteCarloIntegrator
{

#define localEnvMapWidth 16
#define localEnvMapHeight 8
	typedef std::array<float, 128> LocalEnvMap;

public:
	MyPhotonMapper(const Properties &props) : MonteCarloIntegrator(props)
	{
		m_traced_photon_paths = props.getInteger("traced_photon_paths", 1);
		m_number_of_envmaps = props.getInteger("number_of_envmaps", 1);
		std::srand(std::time(0));
		m_photons.reserve(8 * m_traced_photon_paths);
		m_pointCloud_photons.points.reserve(8 * m_traced_photon_paths);
		m_envmaps.reserve(8 * m_number_of_envmaps);
		m_pointCloud_envmaps.points.reserve(8 * m_number_of_envmaps);
	}

	MyPhotonMapper(Stream *stream, InstanceManager *manager) : MonteCarloIntegrator(stream, manager){
		m_traced_photon_paths = stream->readInt();
		m_number_of_envmaps = stream->readInt();
	}

	void serialize(Stream *stream, InstanceManager *manager) const{
		SamplingIntegrator::serialize(stream, manager);
		stream->writeInt(m_traced_photon_paths);
		stream->writeInt(m_number_of_envmaps);
	}

	bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int sensorResID, int samplerResID){
		MonteCarloIntegrator::preprocess(scene, queue, job, sceneResID, sensorResID, samplerResID);

		RayDifferential ray;
		Intersection its;
		const Emitter* e = scene->getEmitters().front().get(); //TODO alle Emitter betrachten statt nur einen

		for (int i = 0; i < m_traced_photon_paths; i++){
			Point2 p(((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX))); //TODO Sampler von mitsuba holen
			Point2 q(((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX)));
			scene->sampleEmitterRay(ray, e, p, q, ray.time);

			Intersection originOnLightSource;
			originOnLightSource.p = ray.o;
			Spectrum throughput = e->eval(originOnLightSource, ray.d); //TODO throughput /= (m_traced_photon_paths * p)  - - mir fehlt die Wahrscheinlichkeit p, mit der die Richtung gesamplet wurde
			bool continuePath = true;
			int pathlength = 0; //f¸r RR oder so, sp‰ter.
			while (continuePath){
				if (scene->rayIntersect(ray, its)){
					pathlength++;
					Photon* p = new Photon(its.p, its.geoFrame, throughput, its.getBSDF(), -ray.d);
					m_photons.push_back(*p); //TODO nicht an spekularen fl‰chen speichern, aber weiterverfolgen?
					Vector3 v(p->position[0], p->position[1], p->position[2]);
					m_pointCloud_photons.points.push_back(v);

					//Weiterverfolgen //TODO mit RR
					const BSDF* bsdf = its.getBSDF();
					BSDFSamplingRecord bRec(its, -ray.d);

					const Point2 samplePoint(((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX)));
					Spectrum bsdfWeight = bsdf->sample(bRec, samplePoint);
					throughput *= bsdfWeight;
					const Vector wo = its.toWorld(bRec.wo);
					ray = Ray(its.p, wo, ray.time);
				}
				else {
					continuePath = false;
				}
			}
			/*if (i == 0){
				Log(EDebug, "First Photon: Energies:");
				for (int sdf = 0; sdf < m_photons.size(); sdf++){
					//FUN FACT: Diese Zeile hier macht den Photonmapper kaputt.
					Log(EDebug, "%i st Energy: Abs: %d, Avg: %d", sdf, m_photons[sdf].energy.abs(), m_photons[sdf].energy.average());
				}
			}*/
		}

		m_kdtree_photons = new PointCloudMap<float, 3>(m_pointCloud_photons);

		fillEnvMaps();

		return true;

	}

	Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const{

		return visualizePhotons(r, rRec);

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

		return bsdfval * closest_photon.energy / dist; //naja damit man halt irgendwas sieht
	}

	void fillEnvMaps(){
		//erstmal nur an Photonen-Stellen, ich weiﬂ grade nicht wo die Kamera ist.
		if (m_number_of_envmaps < m_photons.size()){
			int k = 32;
			for (int i = 0; i < m_number_of_envmaps; i++){ //einfach mal die ersten # photonen
				Photon photon = m_photons[i];

				std::vector<std::pair<std::size_t, float>> indices_distances;
				Vector3 v(photon.position[0], photon.position[1], photon.position[2]);
				m_kdtree_photons->query_knn(k, v, indices_distances); //get k nearest neighbours of photon / cache position

				LocalEnvMap envmapcache;
				envmapcache.fill(0.f);

				float sum = 0;

				if (indices_distances.size() == k){
					for (int j = 0; j < k; j++){
						Photon photon_j = m_photons[indices_distances[j].first];

						Vector dir = photon.surface_normal_frame.toLocal(photon_j.w_i);
						float cos = photon.surface_normal_frame.cosTheta(dir);
						if (cos >= 0 || !photon.bsdf->ESmooth){
							Point2d texCoord = MyEnvMap::directionToTextureRectangular(dir);
							int x = (int)floor(texCoord[0] * localEnvMapWidth);
							int y = (int)floor(texCoord[1] * localEnvMapHeight);
							envmapcache[x + y*localEnvMapWidth] += photon_j.energy.average();
							sum += photon_j.energy.average();
						}
					}
					for (int j = 0; j < 128; j++){
						envmapcache[j] /= sum;
					}
					m_envmaps.push_back(envmapcache);
					m_pointCloud_envmaps.points.push_back(v);
				}//if indices_distances.size() == k
			}//for number_of_envmaps
			m_kdtree_envmaps = new PointCloudMap<float, 3>(m_pointCloud_envmaps);
		}
	}


private:
	/*Nimmt Array mit Wahrscheinlichkeiten, w‰hlt entsprechend zuf‰llig ein K‰stchen, und dann einen zuf‰lligen Punkt in diesem K‰stchen.*/
	Point2f sampleLocalEnvMapTexel(LocalEnvMap em, float& pdf){
		float sum = 0;
		for (int i = 0; i < em.size(); i++){
			sum += em[i];
			if ((double)rand() / (RAND_MAX) < sum){
				float x = ((i%localEnvMapWidth) + (float)rand() / (RAND_MAX))/localEnvMapWidth; //Texel + zuf‰lliges x in diesem Texel, skaliert auf (0,1)
				float y = ((i / localEnvMapWidth) + (float)rand() / (RAND_MAX))/localEnvMapHeight;//Texel.y + zuf‰lliges y in diesem Texel, skaliert auf (0,1)
				pdf = em[i]; //TODO noch iwie durch 4pi teilen damit Fl‰che von ganzer Kugel abgedeckt ist?
				return Point2f(x,y);
			}
		}
		Log(EDebug, "Kein Punkt in der envmap gesamplet. Komisch, eigentlich sollte die Summe am Ende 1 sein.");
		return Point2f(0.f, 0.f);
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
		std::string sdf =closest_photon.energy.toString();
		const char* c = sdf.c_str();
		Log(EDebug, "Spectrum Sum of Values: s[] 0+1+2: %f", closest_photon.energy[0] + closest_photon.energy[1] + closest_photon.energy[2]);
		return bsdfval * closest_photon.energy / dist; //naja damit man halt irgendwas sieht
	}
	

	MTS_DECLARE_CLASS()
protected:
	int m_traced_photon_paths;
	int m_number_of_envmaps;
	std::vector<Photon> m_photons;
	PointCloud<float, 3> m_pointCloud_photons;
	PointCloudMap<float, 3>* m_kdtree_photons;
	std::vector<LocalEnvMap> m_envmaps;
	PointCloud<float, 3> m_pointCloud_envmaps; //noch nicht benutzt
	PointCloudMap<float, 3>* m_kdtree_envmaps; //noch nicht benutzt
};
MTS_IMPLEMENT_CLASS_S(MyPhotonMapper, false, MonteCarloIntegrator)
MTS_EXPORT_PLUGIN(MyPhotonMapper, "My Photon Mapper");
MTS_NAMESPACE_END