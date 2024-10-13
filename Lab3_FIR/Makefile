fir:
	rm -rf xsim.dir/ *.log *.pb *.jou *.wdb
	xvlog -f ./include.rtl.list.xsim ./fir_tb_prev1.v
	xelab -top fir_tb -snapshot fir_tb_elab
	xsim fir_tb_elab -R
