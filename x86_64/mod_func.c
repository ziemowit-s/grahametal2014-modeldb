#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern void _burststim2_reg(void);
extern void _cad_reg(void);
extern void _cagk_reg(void);
extern void _carF_reg(void);
extern void _distca_reg(void);
extern void _distr_reg(void);
extern void _h_reg(void);
extern void _kadist_reg(void);
extern void _kaprox_reg(void);
extern void _kca_reg(void);
extern void _kdrca1_reg(void);
extern void _km_reg(void);
extern void _na3n_reg(void);
extern void _naxn_reg(void);
extern void _nmdaca_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," burststim2.mod");
    fprintf(stderr," cad.mod");
    fprintf(stderr," cagk.mod");
    fprintf(stderr," carF.mod");
    fprintf(stderr," distca.mod");
    fprintf(stderr," distr.mod");
    fprintf(stderr," h.mod");
    fprintf(stderr," kadist.mod");
    fprintf(stderr," kaprox.mod");
    fprintf(stderr," kca.mod");
    fprintf(stderr," kdrca1.mod");
    fprintf(stderr," km.mod");
    fprintf(stderr," na3n.mod");
    fprintf(stderr," naxn.mod");
    fprintf(stderr," nmdaca.mod");
    fprintf(stderr, "\n");
  }
  _burststim2_reg();
  _cad_reg();
  _cagk_reg();
  _carF_reg();
  _distca_reg();
  _distr_reg();
  _h_reg();
  _kadist_reg();
  _kaprox_reg();
  _kca_reg();
  _kdrca1_reg();
  _km_reg();
  _na3n_reg();
  _naxn_reg();
  _nmdaca_reg();
}
