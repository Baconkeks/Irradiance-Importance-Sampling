#include <mitsuba/render/scene.h>
#include <mitsuba/myenvmap.h>

MTS_NAMESPACE_BEGIN
class PathExplicit : public	MonteCarloIntegrator
{
public:
	PathExplicit(const Properties &props) : MonteCarloIntegrator(props)
	{
		m_fix_w_I = props.getFloat("fix_w_I", -1.0f) / 100;
		m_fix_w_E = props.getFloat("fix_w_E", -1.0f) / 100;
		m_rr_prob = props.getFloat("rr_prob", 1.0f) / 100;
		m_use_own_envmap = props.getBoolean("use_own_envmap", false);
		if (m_use_own_envmap){
			m_envmap = new MyEnvMap();
		}
	}

	PathExplicit(Stream *stream, InstanceManager *manager) : MonteCarloIntegrator(stream, manager){
		m_fix_w_I = stream->readFloat();
		m_fix_w_E = stream->readFloat();
		m_rr_prob = stream->readFloat();
		m_use_own_envmap = stream->readBool();
	}

	void serialize(Stream *stream, InstanceManager *manager) const{
		SamplingIntegrator::serialize(stream, manager);
		stream->writeFloat(m_fix_w_I);
		stream->writeFloat(m_fix_w_E);
		stream->writeFloat(m_rr_prob);
		stream->writeBool(m_use_own_envmap);
	}

	Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const{
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
		bool use_fix_w_I = (m_fix_w_I >= 0);
		bool use_fix_w_E = (m_fix_w_E >= 0);
		if (use_fix_w_I){
			w_I = m_fix_w_I;
		}
		if (use_fix_w_E){
			w_E = m_fix_w_E;
		}
		bool use_fix_rr_prob = m_rr_prob >= 0;

		while (!pathTerminated){
			i++;

			/* Check implicit path */
			if (its.isEmitter()){
				if (i > 1){
					//w_I wurde schon in der while-Schleife davor berechnet, weil da die Objekte dafür direkt da waren.
				}
				else {
					if (!use_fix_w_I) w_I = 1;
				}
				Li += w_I*f_X*its.Le(-ray.d);
				pathTerminated = true;
			}
			else {

				/* Make explicit connection */
				const BSDF *bsdf = its.getBSDF(ray); //BSDF bzgl w_o bzw einfallendem Pfad bzw später ausgehendem licht
				bool bsdfSmooth = bsdf->getType() & BSDF::ESmooth;
				DirectSamplingRecord dRec(its);
				if (bsdfSmooth){ //Nur bei nicht-Delta-BSDFs!
					Spectrum value = scene->sampleEmitterDirect(dRec, rRec.nextSample2D());
					if (!value.isZero()){

						BSDFSamplingRecord bRec(its, its.toLocal(dRec.d)); //bRec wird direkt mit den Daten gefüllt, das Sample wurde mit dRec erstellt

						const Spectrum bsdfVal = bsdf->eval(bRec); //Evaluate BSDF*cos(theta)  //theta bzgl w_o
						if (!bsdfVal.isZero()){ //und Geometriebedingungen, hier weggelassen
							float bsdfPdf = bsdf->pdf(bRec); //TODO Spiegel/Glas box nochmal mit EArea und ESolidAngle testen //pdf über solid angle //Wie wahrscheinlich wäre es gewesen, dieses Sample über die normale pdf für Strahlfortsetzen zu erhalten?
							Float lumPdf = scene->pdfEmitterDirect(dRec); //Wie wahrscheinlich wäre es gewesen, dieses Sample zu samplen, hätte man ein Emittersample holen wollen?
							
							//TODO: if (dRec.Measure == EArea) dann muss man noch was machen. Kucken ob das passiert.
							
							float weight = lumPdf*lumPdf / (lumPdf*lumPdf + (bsdfPdf*bsdfPdf));//weight entspricht wahrscheinlich w_E mit beta=2 //TODO Hä?
							if (!use_fix_w_E) w_E = weight;
							Li += w_E*f_X*bsdfVal*value; //evtl fehlen Geometriefaktoren für projected solid angle. grade alles mit normalen solid angles.
						}

						/*
						bsdf->pdf(bRec):
						Compute the probability of sampling bRec.wo (given bRec.wi).
						This method provides access to the probability density that would result when supplying the same BSDF query record to the sample() method.
						It correctly handles changes in probability when only a subset of the components is chosen for sampling
						(this can be done using the BSDFSamplingRecord::component and BSDFSamplingRecord::typeMask fields).
						- kann als 2. Parameter measure erhalten, default: 	EMeasure measure = ESolidAngle
						measure
						Specifies the measure of the component. This is necessary to handle BSDFs, whose components live on spaces with different measures. (E.g. a diffuse material with an ideally smooth dielectric coating).
						*/
					}
				}

				/* Russian Roulette */
				if (i >= m_rrDepth){
					if (use_fix_rr_prob){
						float random = rRec.nextSample1D();
						if (random > m_rr_prob){
							pathTerminated = true;
							return Li;
						}
						else {
							f_X /= m_rr_prob;
						}
					}
					else {
						Spectrum diffRef = bsdf->getDiffuseReflectance(its);
						Spectrum specRef = bsdf->getSpecularReflectance(its);
						float alpha = diffRef.average() + specRef.average();
						if (alpha > 1) alpha = 1;
						float random = rRec.nextSample1D();
						if (random > alpha){
							pathTerminated = true;
							return Li;
						}
						else {
							f_X /= alpha;
						}
					}
				}

				/* Extend Path */
				Float bsdfPdf;
				BSDFSamplingRecord bRec(its, rRec.sampler, ERadiance);
				Intersection previousIts = its;
				Point previousPoint = its.p;
				Spectrum bsdfWeight = bsdf->sample(bRec, bsdfPdf, rRec.nextSample2D());
				if (bsdfWeight.isZero()){ //bsdf->sample == zero  =>  Sampling fehlgeschlagen
					return Li; //Wow. anscheinend passiert das doch öfter mal. Hatte davor nur Spectrum(0) zurückgegeben deswegen bei Veach so komische Schatten hinter den Brettern.
				}
				f_X *= bsdfWeight;
				const Vector wo = its.toWorld(bRec.wo);
				ray = Ray(its.p, wo, ray.time);

				if (scene->rayIntersect(ray, its)){ //TODO: rRec.intersectionExists(ray) besser? Und dann über rRec.its its neu setzen
					/* w_I schon hier setzen, im nächsten while-Durchlauf ist dRec weg.*/
					if (its.isEmitter()){
						dRec.setQuery(ray, its);
						Float lumPdf = scene->pdfEmitterDirect(dRec); //Wie wahrscheinlich wäre es gewesen, dieses Sample zu samplen, hätte man ein Emittersample holen wollen?

						//TODO: if (dRec.Measure == EArea) dann muss man noch was machen. Kucken ob das passiert.
						if (!use_fix_w_I){
							if (bsdfSmooth)	w_I = bsdfPdf*bsdfPdf / (bsdfPdf*bsdfPdf + lumPdf*lumPdf); //Power heuristic beta=2
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
								if (!use_fix_w_I) w_I = bsdfPdf*bsdfPdf / (bsdfPdf*bsdfPdf + lumPdf*lumPdf);
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

	MTS_DECLARE_CLASS()
protected:
	float m_fix_w_I; //fix MIS weight for implicipt paths; -1 => compute w_I
	float m_fix_w_E; //fix MIS weight for explicit paths; -1 => compute w_E
	float m_rr_prob; //fix probability for russian roulette; -1 => compute probability
	bool m_use_own_envmap;
	MyEnvMap *m_envmap;
};
MTS_IMPLEMENT_CLASS_S(PathExplicit, false, MonteCarloIntegrator)
MTS_EXPORT_PLUGIN(PathExplicit, "Path Tracer (explicit)");
MTS_NAMESPACE_END