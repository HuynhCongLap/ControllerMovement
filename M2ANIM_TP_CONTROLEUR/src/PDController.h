
#ifndef PDCONTROLLER_H
#define PDCONTROLLER_H

class PDController {

public:

    // ================================ TODO PARTIE II ==================================== //

    // Constructeur
	PDController(double KpGain=0.0, double KdGain=0.0) : _targetValue(0.0), _targetVelocity(0.0), _previousTargetValue(0.0), _previousCurrentValue(0.0), _kp(KpGain), _kd(KdGain) {}

	// Mise a jour de l'etat du regulateur
	void setTarget(double targetValue) {_previousTargetValue=_targetValue;_targetValue=targetValue;_targetVelocity=_targetValue-_previousTargetValue;}
	double compute (double currentValue);

	// set/get
	void setGains(double newKp,double newKd) {setKpGain(newKp);setKdGain(newKd);}
	void setKpGain (double newKp) {_kp=newKp;}
	double getKpGain() const {return _kp;}
	void setKdGain (double newKd) {_kd=newKd;}
	double getKdGain() const {return _kd;}

protected:

	double _targetValue;
	double _targetVelocity;
	double _previousTargetValue;
	double _previousCurrentValue;

	double _kp;
	double _kd;

	// ================================================================================== //

};

#endif

