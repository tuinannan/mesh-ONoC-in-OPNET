/* Process model C form file: Ip_Switch_mesh.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char Ip_Switch_mesh_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A modeler 7 58EB6791 58EB6791 1 DESKTOP-QABS3U0 gyn 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1bcc 1                                                                                                                                                                                                                                                                                                                                                                                                      ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */

#include "opnet.h"
#include "math.h"



/* 状态转移的宏定义 */
#define  ARRIVAL             (op_intrpt_type () == OPC_INTRPT_STRM)


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





/*定义不同分组的标记*/
#define PATHSETUP_FLAG        0
#define OPTICAL_FLAG          1
#define ACK_FLAG              2

/*输出流的宏定义*/
#define XMT_OUT_STRM          1
#define SINK_OUT_STRM         0

/*定义队列的索引*/
#define SUBQ_OP               0 
 



/*函数声明*/
void ack_arrival_actions(Packet * pkptr);
void optical_arrival_actions(Packet * pkptr,int instrm);
void electronic_arrival_actions(Packet * pkptr,int instrm);

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
	double	                 		OP_length                                       ;
	double	                 		transmission_bandwidth                          ;
	Objid	                  		own_objid                                       ;
	Objid	                  		node_objid                                      ;
	double	                 		inter_check_period                              ;
	int	                    		local_node_address[3]                           ;
	double	                 		S_Number                                        ;
	double	                 		ack_length                                      ;
	double	                 		detector_rate                                   ;
	} Ip_Switch_mesh_state;

#define OP_length               		op_sv_ptr->OP_length
#define transmission_bandwidth  		op_sv_ptr->transmission_bandwidth
#define own_objid               		op_sv_ptr->own_objid
#define node_objid              		op_sv_ptr->node_objid
#define inter_check_period      		op_sv_ptr->inter_check_period
#define local_node_address      		op_sv_ptr->local_node_address
#define S_Number                		op_sv_ptr->S_Number
#define ack_length              		op_sv_ptr->ack_length
#define detector_rate           		op_sv_ptr->detector_rate

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	Ip_Switch_mesh_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((Ip_Switch_mesh_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif


/*定义建链分组到达后的处理行为*/
void electronic_arrival_actions(Packet *pkptr, int instrm)
	{
	Packet *pkptr_temp;
	int ID_NO_temp;
	
	int S_X_temp,S_Y_temp,S_Z_temp;
	int D_X_temp,D_Y_temp,D_Z_temp;
	double elec_power_temp;
	int hop_num;
	
    FIN(electronic_arrival_actions(pkptr, instrm));	
/**/		
	if(instrm == 0)  /*从IP核发送，则直接从输出流发送给下一跳节点*/
		{
		op_pk_send_forced(pkptr, XMT_OUT_STRM);
		}
	else           /*到达目的IP核，记录信息并销毁该建链分组，生成ACK分组，设置相应信息并沿原路返回*/
		{
		op_pk_fd_get(pkptr,ID_NO,&ID_NO_temp);
		
		op_pk_fd_get(pkptr, SOUR_ADDR_FIELD_X,  &S_X_temp);
		
        op_pk_fd_get(pkptr, SOUR_ADDR_FIELD_Y, &S_Y_temp);
		
		op_pk_fd_get(pkptr, SOUR_ADDR_FIELD_Z, &S_Z_temp);
	
	    op_pk_fd_get(pkptr, DEST_ADDR_FIELD_X, &D_X_temp);
    
		op_pk_fd_get(pkptr, DEST_ADDR_FIELD_Y, &D_Y_temp);
		
		op_pk_fd_get(pkptr, DEST_ADDR_FIELD_Z, &D_Z_temp);
		
		op_pk_fd_get(pkptr, ELEC_POWER_FIELD, &elec_power_temp);
		
		op_pk_fd_get(pkptr, HOP_FIELD, &hop_num );
		
		
		
		op_pk_destroy(pkptr);
		
		
		pkptr_temp = op_pk_create(ack_length);
		
		op_pk_encap_flag_set(pkptr_temp, ACK_FLAG);
		
	    op_pk_fd_set(pkptr_temp, ID_NO, OPC_FIELD_TYPE_INTEGER, ID_NO_temp, 15);
		 
	    op_pk_fd_set(pkptr_temp, SOUR_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, S_X_temp, 4);
		   
        op_pk_fd_set(pkptr_temp, SOUR_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, S_Y_temp, 4);
		 
	    op_pk_fd_set(pkptr_temp, SOUR_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, S_Z_temp, 4);
		
		op_pk_fd_set(pkptr_temp, DEST_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, D_X_temp, 4);
		   
	    op_pk_fd_set(pkptr_temp, DEST_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, D_Y_temp, 4);
		
		op_pk_fd_set(pkptr_temp, DEST_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, D_Z_temp, 4);
		
		op_pk_fd_set(pkptr_temp, ELEC_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, elec_power_temp, 20);
		
		op_pk_fd_set(pkptr_temp, HOP_FIELD, OPC_FIELD_TYPE_INTEGER, hop_num, 5);
		
		
		
		op_pk_total_size_set(pkptr_temp, ack_length);
			
		op_pk_send_forced(pkptr_temp, XMT_OUT_STRM);
		
		}
	FOUT;
	}









/*定义信息分组到达后的处理行为*/
void optical_arrival_actions(Packet *pkptr, int instrm)
	{
	
	FIN(optical_arrival_actions(pkptr, instrm));
	
	if(instrm == 0)                 /*从IP核发送，则存入队列，等待ACK分组*/
		{
		op_subq_pk_insert(SUBQ_OP, pkptr, OPC_QPOS_TAIL);
		}
	else                          /*到达目的IP核，直接发送到sink*/
		{
		op_pk_send_forced(pkptr, SINK_OUT_STRM);
		}
	
	FOUT;
	}








/*定义ACK分组到达的处理行为*/
void ack_arrival_actions(Packet *pkptr)
	{
	 Packet  *pkptr_optical_temp;
	 double  elec_power_temp;
	 int    hop_num;
	
	
	 FIN(ack_arrival_actions(pkptr));

	  /*得到记录信息*/
	  op_pk_fd_get(pkptr, ELEC_POWER_FIELD, &elec_power_temp);
	  op_pk_fd_get(pkptr, HOP_FIELD, &hop_num );
	
      /*把得到的信息写入信息分组，并发送信息分组，然后销毁ACK分组*/
	  
      pkptr_optical_temp = op_subq_pk_remove(SUBQ_OP, OPC_QPOS_HEAD);
	  
	  op_pk_fd_set(pkptr_optical_temp, ELEC_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, elec_power_temp,20);
		
	  op_pk_fd_set(pkptr_optical_temp, HOP_FIELD, OPC_FIELD_TYPE_INTEGER, hop_num, 5);
	  op_pk_send_delayed(pkptr_optical_temp, XMT_OUT_STRM, OP_length/transmission_bandwidth-32/10000000);	

	  op_pk_destroy(pkptr);
	
	
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
	void Ip_Switch_mesh (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_Ip_Switch_mesh_init (int * init_block_ptr);
	void _op_Ip_Switch_mesh_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_Ip_Switch_mesh_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_Ip_Switch_mesh_alloc (VosT_Obtype, int);
	void _op_Ip_Switch_mesh_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
Ip_Switch_mesh (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (Ip_Switch_mesh ());

		{
		/* Temporary Variables */
		Packet * pkptr;
		int instrm;
		/* End of Temporary Variables */


		FSM_ENTER ("Ip_Switch_mesh")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (init) enter executives **/
			FSM_STATE_ENTER_FORCED_NOLABEL (0, "init", "Ip_Switch_mesh [init enter execs]")
				FSM_PROFILE_SECTION_IN ("Ip_Switch_mesh [init enter execs]", state0_enter_exec)
				{
				/* 得到模块的对象ID*/
				own_objid = op_id_self ();
				/* 得到节点的对象ID   */
				node_objid = op_topo_parent(own_objid);
				
				/*读取节点的属性*/
				op_ima_obj_attr_get(node_objid, "Node_Address_X", &local_node_address[0]);
				op_ima_obj_attr_get(node_objid, "Node_Address_Y", &local_node_address[1]);
				op_ima_obj_attr_get(node_objid, "Node_Address_Z", &local_node_address[2]);
				op_ima_obj_attr_get(node_objid, "Detector_Rate(Gbps)", &detector_rate);
				
				
				
				/*读取仿真参数配置*/
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Optical Fixed Packet Length(bit)", &OP_length);
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Transmission Bandwidth (Gbps)", &transmission_bandwidth);
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Roll Around Period(ns)", &inter_check_period);
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Ack Length(bit)", &ack_length);
				
				
				
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** state (init) exit executives **/
			FSM_STATE_EXIT_FORCED (0, "init", "Ip_Switch_mesh [init exit execs]")


			/** state (init) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "init", "idle", "tr_0", "Ip_Switch_mesh [init -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (idle) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "idle", state1_enter_exec, "Ip_Switch_mesh [idle enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"Ip_Switch_mesh")


			/** state (idle) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "idle", "Ip_Switch_mesh [idle exit execs]")


			/** state (idle) transition processing **/
			FSM_PROFILE_SECTION_IN ("Ip_Switch_mesh [idle trans conditions]", state1_trans_conds)
			FSM_INIT_COND (ARRIVAL)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("idle")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 2, state2_enter_exec, ;, "ARRIVAL", "", "idle", "arrival", "tr_1", "Ip_Switch_mesh [idle -> arrival : ARRIVAL / ]")
				FSM_CASE_TRANSIT (1, 1, state1_enter_exec, ;, "default", "", "idle", "idle", "tr_9", "Ip_Switch_mesh [idle -> idle : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (arrival) enter executives **/
			FSM_STATE_ENTER_FORCED (2, "arrival", state2_enter_exec, "Ip_Switch_mesh [arrival enter execs]")
				FSM_PROFILE_SECTION_IN ("Ip_Switch_mesh [arrival enter execs]", state2_enter_exec)
				{
				 /*得到输入流上的分组*/
				instrm = op_intrpt_strm();
				
				pkptr = op_pk_get(instrm);
				
				
				/*判断分组的标记，调用不同的函数*/
				
				if (op_pk_encap_flag_is_set(pkptr, PATHSETUP_FLAG))
					electronic_arrival_actions(pkptr, instrm);
				
				
				else if (op_pk_encap_flag_is_set(pkptr, ACK_FLAG))
					ack_arrival_actions(pkptr);
				 
				else if(op_pk_encap_flag_is_set(pkptr, OPTICAL_FLAG))
					optical_arrival_actions(pkptr,instrm);
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** state (arrival) exit executives **/
			FSM_STATE_EXIT_FORCED (2, "arrival", "Ip_Switch_mesh [arrival exit execs]")


			/** state (arrival) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "arrival", "idle", "tr_4", "Ip_Switch_mesh [arrival -> idle : default / ]")
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"Ip_Switch_mesh")
		}
	}




void
_op_Ip_Switch_mesh_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
	/* No Diagnostic Block */
	}




void
_op_Ip_Switch_mesh_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_Ip_Switch_mesh_terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_Ip_Switch_mesh_svar function. */
#undef OP_length
#undef transmission_bandwidth
#undef own_objid
#undef node_objid
#undef inter_check_period
#undef local_node_address
#undef S_Number
#undef ack_length
#undef detector_rate

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_Ip_Switch_mesh_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_Ip_Switch_mesh_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (Ip_Switch_mesh)",
		sizeof (Ip_Switch_mesh_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_Ip_Switch_mesh_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	Ip_Switch_mesh_state * ptr;
	FIN_MT (_op_Ip_Switch_mesh_alloc (obtype))

	ptr = (Ip_Switch_mesh_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "Ip_Switch_mesh [init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_Ip_Switch_mesh_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	Ip_Switch_mesh_state		*prs_ptr;

	FIN_MT (_op_Ip_Switch_mesh_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (Ip_Switch_mesh_state *)gen_ptr;

	if (strcmp ("OP_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->OP_length);
		FOUT
		}
	if (strcmp ("transmission_bandwidth" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->transmission_bandwidth);
		FOUT
		}
	if (strcmp ("own_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->own_objid);
		FOUT
		}
	if (strcmp ("node_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_objid);
		FOUT
		}
	if (strcmp ("inter_check_period" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->inter_check_period);
		FOUT
		}
	if (strcmp ("local_node_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->local_node_address);
		FOUT
		}
	if (strcmp ("S_Number" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->S_Number);
		FOUT
		}
	if (strcmp ("ack_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ack_length);
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

