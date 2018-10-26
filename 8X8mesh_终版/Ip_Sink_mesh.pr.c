/* Process model C form file: Ip_Sink_mesh.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char Ip_Sink_mesh_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A modeler 7 5727108C 5727108C 1 CN13549845642 Administrator 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1bcc 1                                                                                                                                                                                                                                                                                                                                                                                              ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */


#include "opnet.h"
#include "math.h"
#include <stdio.h>


/*定义状态转移条件*/
#define  ARRIVAL     (op_intrpt_type() == OPC_INTRPT_STRM)
#define  END_SIM     (op_intrpt_type() == OPC_INTRPT_ENDSIM)



/*定义不同分组的标记*/
#define PATHSETUP_FLAG        0
#define OPTICAL_FLAG          1
#define ACK_FLAG              2


/*定义包的域*/
#define ID_NO				  0	          
#define DEST_ADDR_FIELD_X     1
#define DEST_ADDR_FIELD_Y     2
#define DEST_ADDR_FIELD_Z     3
#define SOUR_ADDR_FIELD_X	  4
#define SOUR_ADDR_FIELD_Y	  5
#define SOUR_ADDR_FIELD_Z	  6


/*  新添加的包的域  */
#define ELEC_POWER_FIELD              7                    
#define OPT_POWER_FIELD               8                    
#define OPT_LOSS_FIELD                9                   
#define HOP_FIELD                     10     

/*定义统计变量*/
double   total_ete_delay = 0;
long int rvd_pkts = 0;

double Max_Loss_Optic = 0;
long int  Max_Hop_Optic = 0;
double Total_Power_Disspation_Electronic = 0;
double Total_Power_Disspation_Optic  = 0;
double Total_Loss_Optic = 0;


/*函数声明*/
static void record_stats();

/* End of Header Block */

#if !defined (VOSD_NO_FIN)
#undef	BIN
#undef	BOUT
#define	BIN		FIN_LOCAL_FIELD(_op_last_line_passed) = __LINE__ - _op_block_origin;
#define	BOUT	BIN
#define	BINIT	FIN_LOCAL_FIELD(_op_last_line_passed) = 0; _op_block_origin = __LINE__;
#else
#define	BINIT
#endif /* #if !defined (VOSD_NO_FIN) */



/* State variable definitions */
typedef struct
	{
	/* Internal state tracking for FSM */
	FSM_SYS_STATE
	/* State Variables */
	Objid	                  		surr_mod_objid                                  ;
	Objid	                  		surr_node_objid                                 ;
	int	                    		END_PER                                         ;
	double	                 		Offered_load                                    ;
	Stathandle	             		ete_gsh                                         ;
	double	                 		ETE_Delay                                       ;
	double	                 		modulator_loss                                  ;
	double	                 		modulator_pow                                   ;
	double	                 		modulator_rate                                  ;
	double	                 		detector_pow                                    ;
	double	                 		detector_sen                                    ;
	double	                 		detector_loss                                   ;
	double	                 		OP_length                                       ;
	double	                 		detector_rate                                   ;
	} Ip_Sink_mesh_state;

#define surr_mod_objid          		op_sv_ptr->surr_mod_objid
#define surr_node_objid         		op_sv_ptr->surr_node_objid
#define END_PER                 		op_sv_ptr->END_PER
#define Offered_load            		op_sv_ptr->Offered_load
#define ete_gsh                 		op_sv_ptr->ete_gsh
#define ETE_Delay               		op_sv_ptr->ETE_Delay
#define modulator_loss          		op_sv_ptr->modulator_loss
#define modulator_pow           		op_sv_ptr->modulator_pow
#define modulator_rate          		op_sv_ptr->modulator_rate
#define detector_pow            		op_sv_ptr->detector_pow
#define detector_sen            		op_sv_ptr->detector_sen
#define detector_loss           		op_sv_ptr->detector_loss
#define OP_length               		op_sv_ptr->OP_length
#define detector_rate           		op_sv_ptr->detector_rate

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	Ip_Sink_mesh_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((Ip_Sink_mesh_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif

/*将统计量写入标量文件*/
void   record_stats()	  
	{
	FIN(record_stats());
	op_stat_scalar_write("ETE Delay(ns)", (double)total_ete_delay/rvd_pkts);
	op_stat_scalar_write("kgc Packet Received", rvd_pkts);
	op_stat_scalar_write("Offered Load", Offered_load);
	op_stat_scalar_write("Average electronic disspation", (double)Total_Power_Disspation_Electronic/rvd_pkts);
	op_stat_scalar_write("Average optic disspation",      (double)Total_Power_Disspation_Optic/rvd_pkts);   
	op_stat_scalar_write("Avetage optic loss",            (double)Total_Loss_Optic/rvd_pkts);
	op_stat_scalar_write("Max optic loss",                (double)Max_Loss_Optic);
	op_stat_scalar_write("Max hop",                       (int)Max_Hop_Optic);
	
	printf("收到的包总数：%d\n",rvd_pkts);
	printf("ETE Delay：%f\n",(double)total_ete_delay/rvd_pkts);
	FOUT;
	}

/* End of Function Block */

/* Undefine optional tracing in FIN/FOUT/FRET */
/* The FSM has its own tracing code and the other */
/* functions should not have any tracing.		  */
#undef FIN_TRACING
#define FIN_TRACING

#undef FOUTRET_TRACING
#define FOUTRET_TRACING

#if defined (__cplusplus)
extern "C" {
#endif
	void Ip_Sink_mesh (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_Ip_Sink_mesh_init (int * init_block_ptr);
	void _op_Ip_Sink_mesh_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_Ip_Sink_mesh_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_Ip_Sink_mesh_alloc (VosT_Obtype, int);
	void _op_Ip_Sink_mesh_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
Ip_Sink_mesh (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (Ip_Sink_mesh ());

		{
		/* Temporary Variables */
		Packet*		pkptr;
		/* 定义统计变量 */
		
		double Power_Disspation_Electronic = 0;
		double Power_Disspation_Optic  = 0;
		double Loss_Optic = 0;
		
		int    Hop_Optic = 0;
		/* End of Temporary Variables */


		FSM_ENTER ("Ip_Sink_mesh")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (init) enter executives **/
			FSM_STATE_ENTER_FORCED_NOLABEL (0, "init", "Ip_Sink_mesh [init enter execs]")
				FSM_PROFILE_SECTION_IN ("Ip_Sink_mesh [init enter execs]", state0_enter_exec)
				{
				/* 得到模块的对象ID*/
				surr_mod_objid = op_id_self();
				/* 得到节点的对象ID  */
				surr_node_objid = op_topo_parent(surr_mod_objid);
				
				op_ima_obj_attr_get(surr_node_objid, "Collect_Flag", &END_PER);
				
				
				
				/*    得到IP核能耗的相关参数（Modelator  及 Detector  ） */
				
				op_ima_obj_attr_get(surr_node_objid, "Modulator_Insertion_Loss(dB)",&modulator_loss);
				op_ima_obj_attr_get(surr_node_objid, "Modulator_Power(J/bit)",      &modulator_pow);
				op_ima_obj_attr_get(surr_node_objid, "Modulator_Rate(Gbps)",        &modulator_rate);
				op_ima_obj_attr_get(surr_node_objid, "Detector_Power(J/bit)",       &detector_pow);
				op_ima_obj_attr_get(surr_node_objid, "Detector_Sensitivity(dBm)",   &detector_sen);
				op_ima_obj_attr_get(surr_node_objid, "Detector_Loss(dB)",           &detector_loss);
				op_ima_obj_attr_get(surr_node_objid, "Detector_Rate(Gbps)", &detector_rate);
				
				
				
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Offered Load", &Offered_load);
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Optical Fixed Packet Length(bit)", &OP_length);
				
				
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** state (init) exit executives **/
			FSM_STATE_EXIT_FORCED (0, "init", "Ip_Sink_mesh [init exit execs]")


			/** state (init) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "init", "idle", "tr_0", "Ip_Sink_mesh [init -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (idle) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "idle", state1_enter_exec, "Ip_Sink_mesh [idle enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"Ip_Sink_mesh")


			/** state (idle) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "idle", "Ip_Sink_mesh [idle exit execs]")


			/** state (idle) transition processing **/
			FSM_PROFILE_SECTION_IN ("Ip_Sink_mesh [idle trans conditions]", state1_trans_conds)
			FSM_INIT_COND (ARRIVAL)
			FSM_TEST_COND (END_SIM)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("idle")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 2, state2_enter_exec, ;, "ARRIVAL", "", "idle", "pk_destroy", "tr_3", "Ip_Sink_mesh [idle -> pk_destroy : ARRIVAL / ]")
				FSM_CASE_TRANSIT (1, 3, state3_enter_exec, ;, "END_SIM", "", "idle", "stati_collect", "tr_5", "Ip_Sink_mesh [idle -> stati_collect : END_SIM / ]")
				FSM_CASE_TRANSIT (2, 1, state1_enter_exec, ;, "default", "", "idle", "idle", "tr_6", "Ip_Sink_mesh [idle -> idle : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (pk_destroy) enter executives **/
			FSM_STATE_ENTER_FORCED (2, "pk_destroy", state2_enter_exec, "Ip_Sink_mesh [pk_destroy enter execs]")
				FSM_PROFILE_SECTION_IN ("Ip_Sink_mesh [pk_destroy enter execs]", state2_enter_exec)
				{
				/*对信息分组进行统计，并记录数据*/
				pkptr = op_pk_get(op_intrpt_strm()); 
				
				if (op_pk_encap_flag_is_set(pkptr, OPTICAL_FLAG))
					{
					 op_pk_fd_get(pkptr, ELEC_POWER_FIELD, &Power_Disspation_Electronic);
					 op_pk_fd_get(pkptr, OPT_POWER_FIELD,  &Power_Disspation_Optic);
					 op_pk_fd_get(pkptr, OPT_LOSS_FIELD, &Loss_Optic);
					 Loss_Optic = Loss_Optic + modulator_loss + detector_loss;
				     Power_Disspation_Optic = Power_Disspation_Optic + modulator_pow * OP_length + detector_pow * OP_length;
				
					 op_pk_fd_get(pkptr, HOP_FIELD, &Hop_Optic);
				 
					 Total_Power_Disspation_Electronic += Power_Disspation_Electronic;
					 Total_Power_Disspation_Optic += Power_Disspation_Optic;
					 Total_Loss_Optic += Loss_Optic;
					 if (Max_Loss_Optic < Loss_Optic)	  /*求最大损耗*/	 
				         {
						  Max_Loss_Optic = Loss_Optic;
						  }
					 if(Max_Hop_Optic < Hop_Optic)
					 
					     {
						  Max_Hop_Optic = Hop_Optic;
						  
						  }                     
						
					      
					     total_ete_delay += op_sim_time() - op_pk_creation_time_get(pkptr) + OP_length/detector_rate;// 81.92 
					
					     rvd_pkts++;
				
					}
				
				
				
				/*最后销毁信息分组*/
				op_pk_destroy (pkptr);
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** state (pk_destroy) exit executives **/
			FSM_STATE_EXIT_FORCED (2, "pk_destroy", "Ip_Sink_mesh [pk_destroy exit execs]")


			/** state (pk_destroy) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "pk_destroy", "idle", "tr_4", "Ip_Sink_mesh [pk_destroy -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (stati_collect) enter executives **/
			FSM_STATE_ENTER_UNFORCED (3, "stati_collect", state3_enter_exec, "Ip_Sink_mesh [stati_collect enter execs]")
				FSM_PROFILE_SECTION_IN ("Ip_Sink_mesh [stati_collect enter execs]", state3_enter_exec)
				{
				if (END_PER == 1)
					record_stats();
				}
				FSM_PROFILE_SECTION_OUT (state3_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (7,"Ip_Sink_mesh")


			/** state (stati_collect) exit executives **/
			FSM_STATE_EXIT_UNFORCED (3, "stati_collect", "Ip_Sink_mesh [stati_collect exit execs]")


			/** state (stati_collect) transition processing **/
			FSM_TRANSIT_MISSING ("stati_collect")
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"Ip_Sink_mesh")
		}
	}




void
_op_Ip_Sink_mesh_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
	/* No Diagnostic Block */
	}




void
_op_Ip_Sink_mesh_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_Ip_Sink_mesh_terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_Ip_Sink_mesh_svar function. */
#undef surr_mod_objid
#undef surr_node_objid
#undef END_PER
#undef Offered_load
#undef ete_gsh
#undef ETE_Delay
#undef modulator_loss
#undef modulator_pow
#undef modulator_rate
#undef detector_pow
#undef detector_sen
#undef detector_loss
#undef OP_length
#undef detector_rate

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_Ip_Sink_mesh_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_Ip_Sink_mesh_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (Ip_Sink_mesh)",
		sizeof (Ip_Sink_mesh_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_Ip_Sink_mesh_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	Ip_Sink_mesh_state * ptr;
	FIN_MT (_op_Ip_Sink_mesh_alloc (obtype))

	ptr = (Ip_Sink_mesh_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "Ip_Sink_mesh [init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_Ip_Sink_mesh_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	Ip_Sink_mesh_state		*prs_ptr;

	FIN_MT (_op_Ip_Sink_mesh_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (Ip_Sink_mesh_state *)gen_ptr;

	if (strcmp ("surr_mod_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->surr_mod_objid);
		FOUT
		}
	if (strcmp ("surr_node_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->surr_node_objid);
		FOUT
		}
	if (strcmp ("END_PER" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->END_PER);
		FOUT
		}
	if (strcmp ("Offered_load" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->Offered_load);
		FOUT
		}
	if (strcmp ("ete_gsh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ete_gsh);
		FOUT
		}
	if (strcmp ("ETE_Delay" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ETE_Delay);
		FOUT
		}
	if (strcmp ("modulator_loss" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->modulator_loss);
		FOUT
		}
	if (strcmp ("modulator_pow" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->modulator_pow);
		FOUT
		}
	if (strcmp ("modulator_rate" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->modulator_rate);
		FOUT
		}
	if (strcmp ("detector_pow" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->detector_pow);
		FOUT
		}
	if (strcmp ("detector_sen" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->detector_sen);
		FOUT
		}
	if (strcmp ("detector_loss" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->detector_loss);
		FOUT
		}
	if (strcmp ("OP_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->OP_length);
		FOUT
		}
	if (strcmp ("detector_rate" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->detector_rate);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

