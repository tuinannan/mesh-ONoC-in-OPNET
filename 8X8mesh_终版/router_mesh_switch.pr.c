/* Process model C form file: router_mesh_switch.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char router_mesh_switch_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A modeler 7 59031117 59031117 1 DESKTOP-UAP8N2N tuinannan 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1bcc 1                                                                                                                                                                                                                                                                                                                                                                                                ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */

#include "opnet.h"
#include "math.h"

/* 定义中断码*/
#define ROLLING_CHECK         0

/*宏定义状态转移条件 */
#define ARRIVAL_ACT           (op_intrpt_type() == OPC_INTRPT_STRM)
#define ROLLING	              (op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code() == ROLLING_CHECK)

/*包域的宏定义，包括ID号，源目的地址域，电能耗域，光能耗域，光损耗域，跳数域*/
#define ID_NO				          0	          
#define DEST_ADDR_FIELD_X             1
#define DEST_ADDR_FIELD_Y             2
#define DEST_ADDR_FIELD_Z             3
#define SOUR_ADDR_FIELD_X	          4
#define SOUR_ADDR_FIELD_Y	          5
#define SOUR_ADDR_FIELD_Z	          6

#define ELEC_POWER_FIELD              7
#define OPT_POWER_FIELD               8
#define OPT_LOSS_FIELD                9
#define HOP_FIELD                     10


/*包类型的标记，电建链包、光包、ACK包*/
#define PATHSETUP_FLAG        0
#define OPTICAL_FLAG          1
#define ACK_FLAG              2

/*mesh_64*/
#define dim_of_Xrouter          8
#define dim_of_Yrouter          8
#define max_IP_num_of_router    1
#define port_out_num            5

/*用来标记是否连接IP核*/
#define NO_CORE_CONNECTTED      0

/* 5*5 photonic switch,0号端口对应的是west，1号端口对应的是north，*/
/* 2号端口对应的是east，3号端口对应的是south，4号端口对应的是local*/
/*以下宏定义用于统计端口到端口之间的微环的个数和端口对之间的损耗*/
#define port0_to_port0  0                      
#define port0_to_port1  1                 
#define port0_to_port2  2                    
#define port0_to_port3  3
#define port0_to_port4  4
#define port1_to_port0  10
#define port1_to_port1  11
#define port1_to_port2  12
#define port1_to_port3  13
#define port1_to_port4  14
#define port2_to_port0  20
#define port2_to_port1  21
#define port2_to_port2  22
#define port2_to_port3  23
#define port2_to_port4  24
#define port3_to_port0  30
#define port3_to_port1  31
#define port3_to_port2  32
#define port3_to_port3  33
#define port3_to_port4  34
#define port4_to_port0  40
#define port4_to_port1  41
#define port4_to_port2  42
#define port4_to_port3  43
#define port4_to_port4  44

/*以下的长度是指端口对之间的波导长度*/                     
#define port0_to_port1_length  0.0002               
#define port0_to_port2_length  0.00026                    
#define port0_to_port3_length  0.00024
#define port0_to_port4_length  0.00014
#define port1_to_port0_length  0.00028
#define port1_to_port2_length  0.00018
#define port1_to_port3_length  0.0002
#define port1_to_port4_length  0.00026
#define port2_to_port0_length  0.0002
#define port2_to_port1_length  0.00024
#define port2_to_port3_length  0.00012
#define port2_to_port4_length  0.00022
#define port3_to_port0_length  0.0002
#define port3_to_port1_length  0.00028
#define port3_to_port2_length  0.0001
#define port3_to_port4_length  0.00026
#define port4_to_port0_length  0.00014
#define port4_to_port1_length  0.00014
#define port4_to_port2_length  0.00024
#define port4_to_port3_length  0.00022


/*用来表示队列状态的变量，1代表锁，0代表未锁*/
int SQ_Lock[dim_of_Xrouter][dim_of_Yrouter][max_IP_num_of_router];


/*用来表示端口状态的变量，1代表锁，0代表未锁*/
int F_Port_Lock[dim_of_Xrouter][dim_of_Yrouter][port_out_num];


/*端口信息，包括端口的输入流号，输出流号，以及相连的电链路的长度和波导的损耗值*/
typedef struct
	{
	int    outstrm_index;
	int    instrm_index;
	double elink_length;
	double olink_length;
	double olink_bending_loss;
	double olink_crossing_loss;
	}Port_Info;

/*用来存放路径信息以及微环刚置成on的时间*/
typedef struct
	{
	int packet_ID;
	int port_in;
	int port_out;
	double ring_start_on_time;
	}Trace_Info;

/*用来存放轮询成功的那些分组所在的队列和输出端口*/
typedef struct
	{
	int subq_in;
	int port_out;
	}candidate_Info;

/*作为optical_statistics_get函数的返回变量*/
typedef struct
	{
	double power;
	double loss;
	}Optical_Stat;

/*函数声明*/
void ack_arr_actions(Packet *pkptr);
void optical_arr_actions(Packet *pkptr);
void electronic_arr_actions(Packet *pkptr, int instrm);
void rolling_check_func();
static int mesh_routing(Packet *pkptr);
void candidate_list_rolling();
void candidate_chosen_to_send(int list_len, int i);
static double electrical_power_calculate(int port_in, int port_out, int flag);
void list_candidate_empty_set();
static Optical_Stat optical_statistics_get(int port_in, int port_out, double ring_on_time);

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
	int	                    		local_node_address[3]                           ;
	List *	                 		lptr_pathsetup_info                             ;
	Port_Info *	            		port_info                                       ;
	double	                 		inter_check_period                              ;
	double	                 		transmission_bandwidth                          ;
	double	                 		OP_length                                       ;
	List *	                 		lptr_candidate_info[5]                          ;
	double	                 		port_num                                        ;
	double	                 		ring_drop_loss                                  ;
	double	                 		ring_dynamic_power                              ;
	double	                 		ring_static_power                               ;
	double	                 		ring_through_loss                               ;
	double	                 		router_power                                    ;
	double	                 		wire_length                                     ;
	double	                 		wire_propagation_rate                           ;
	double	                 		wire_power                                      ;
	double	                 		waveguide_length                                ;
	double	                 		waveguide_propagation_rate                      ;
	double	                 		waveguide_propagation_loss                      ;
	double	                 		waveguide_bending_loss                          ;
	double	                 		waveguide_crossing_loss                         ;
	double	                 		data_rate                                       ;
	double	                 		path_setup_length                               ;
	double	                 		ack_length                                      ;
	double	                 		dim_of_mesh                                     ;
	double	                 		crossbar_power                                  ;
	int	                    		ip_num_of_this_router                           ;
	double	                 		ring_switch_time                                ;
	double	                 		bending_loss                                    ;
	double	                 		crossing_loss                                   ;
	} router_mesh_switch_state;

#define surr_mod_objid          		op_sv_ptr->surr_mod_objid
#define surr_node_objid         		op_sv_ptr->surr_node_objid
#define local_node_address      		op_sv_ptr->local_node_address
#define lptr_pathsetup_info     		op_sv_ptr->lptr_pathsetup_info
#define port_info               		op_sv_ptr->port_info
#define inter_check_period      		op_sv_ptr->inter_check_period
#define transmission_bandwidth  		op_sv_ptr->transmission_bandwidth
#define OP_length               		op_sv_ptr->OP_length
#define lptr_candidate_info     		op_sv_ptr->lptr_candidate_info
#define port_num                		op_sv_ptr->port_num
#define ring_drop_loss          		op_sv_ptr->ring_drop_loss
#define ring_dynamic_power      		op_sv_ptr->ring_dynamic_power
#define ring_static_power       		op_sv_ptr->ring_static_power
#define ring_through_loss       		op_sv_ptr->ring_through_loss
#define router_power            		op_sv_ptr->router_power
#define wire_length             		op_sv_ptr->wire_length
#define wire_propagation_rate   		op_sv_ptr->wire_propagation_rate
#define wire_power              		op_sv_ptr->wire_power
#define waveguide_length        		op_sv_ptr->waveguide_length
#define waveguide_propagation_rate		op_sv_ptr->waveguide_propagation_rate
#define waveguide_propagation_loss		op_sv_ptr->waveguide_propagation_loss
#define waveguide_bending_loss  		op_sv_ptr->waveguide_bending_loss
#define waveguide_crossing_loss 		op_sv_ptr->waveguide_crossing_loss
#define data_rate               		op_sv_ptr->data_rate
#define path_setup_length       		op_sv_ptr->path_setup_length
#define ack_length              		op_sv_ptr->ack_length
#define dim_of_mesh             		op_sv_ptr->dim_of_mesh
#define crossbar_power          		op_sv_ptr->crossbar_power
#define ip_num_of_this_router   		op_sv_ptr->ip_num_of_this_router
#define ring_switch_time        		op_sv_ptr->ring_switch_time
#define bending_loss            		op_sv_ptr->bending_loss
#define crossing_loss           		op_sv_ptr->crossing_loss

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	router_mesh_switch_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((router_mesh_switch_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif


/*电建链包到达时,通过比较输入流号和端口的流号，将其插入到相应的队列中*/
void electronic_arr_actions(Packet *pkptr, int instrm)
{
	
    int i = -1;
    
	FIN(electronic_arr_actions(pkptr,instrm));
	
	
	
	for (i = 0; i < port_num; i++)
		{
		
		if (port_info[i].instrm_index== instrm) 
			{
			//printf("ok\n");
			
			op_subq_pk_insert(i, pkptr, OPC_QPOS_TAIL);

			break;
			
			}/*end if(port_info[i].instrm_index == instrm) and jump out of the loop*/
		else
			{
			continue;
			}/*it is not the corresponding instrm, continue the loop*/
			
		}
	
	FOUT;

}






/*ack到达时，要完成两项任务
	一：将当前的仿真时间插入到建链信息表中，用来标记微环谐振器的处于on的开始时间。
	二：通过查建链信息表，将ack分组从相应的端口发送出去 */    
void ack_arr_actions(Packet *pkptr)
{
	
    int packet_ID_temp = -1;
	int list_len = -1;
	int i = -1;
	double elec_power_field_temp = 0.0;
	double elec_power_this_jump  = 0.0;
	
	Trace_Info *trace_info_ptr_temp;
	
	FIN(ack_arr_actions(pkptr));
	
	op_pk_fd_get(pkptr, ID_NO, &packet_ID_temp);

	list_len = op_prg_list_size(lptr_pathsetup_info);
	
	for (i = 0; i < list_len; i++)
		{
		trace_info_ptr_temp = (Trace_Info*)op_prg_list_access(lptr_pathsetup_info, i);
		
		if (trace_info_ptr_temp->packet_ID == packet_ID_temp)

		  {
		   trace_info_ptr_temp->ring_start_on_time = op_sim_time();                   /*修改建链表中的ring_start_on_time，首先将原来的信息复制到trace_info_ptr_temp中*/
		                                                                             /*然后记录当前的时间，并销毁原来的信息，最后将trace_info_ptr_temp插入到建链信息表中*/
		   	   /*计算ACK的上一跳能耗，并更新到分组的能耗域*/
		   op_pk_fd_get(pkptr, ELEC_POWER_FIELD, &elec_power_field_temp);
		   elec_power_this_jump  = electrical_power_calculate(trace_info_ptr_temp->port_out, trace_info_ptr_temp->port_in, ACK_FLAG);
		   elec_power_field_temp = elec_power_field_temp + elec_power_this_jump;
		   op_pk_fd_set(pkptr, ELEC_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, elec_power_field_temp, 20);
		   
		   
		   /*根据输入端口的不同来选择发送时是否有delay，如果输入端口是连接IP核的端口，用op_pk_send_forced，并跳出swtich语句*/
		   /*反之，如果输入端口是连接其他Switch的端口,用op_pk_send_delayed，并跳出swtich语句*/
		   if (ip_num_of_this_router != NO_CORE_CONNECTTED)
			   {
			   switch (trace_info_ptr_temp->port_in)
				   {
				   case 0 : op_pk_send_delayed(pkptr, port_info[trace_info_ptr_temp->port_in].outstrm_index, inter_check_period/3-32/10000000);break;
					 
			       case 1 : op_pk_send_delayed(pkptr, port_info[trace_info_ptr_temp->port_in].outstrm_index, inter_check_period/3-32/10000000);break;
				
			       case 2 : op_pk_send_delayed(pkptr, port_info[trace_info_ptr_temp->port_in].outstrm_index, inter_check_period/3-32/10000000);break;
				 
			       case 3 : op_pk_send_delayed(pkptr, port_info[trace_info_ptr_temp->port_in].outstrm_index, inter_check_period/3-32/10000000);break;
			
			       case 4 : op_pk_send_forced(pkptr, port_info[trace_info_ptr_temp->port_in].outstrm_index);break;
			   
			       default: op_prg_odb_print_major("error,undefined port_in", OPC_NIL);
				   }
			   
			    }/*end if(ip_num_of_this_router!= NO_CORE_CONNECTTED)*/
		    else
			   {
			   op_pk_send_delayed(pkptr, port_info[trace_info_ptr_temp->port_in].outstrm_index, inter_check_period/3-32/10000000);
			   } /*end else,and the corresponding if is if (ip_num_of_this_router != NO_CORE_CONNECTTED)*/    

			break;/*jump out of loop*/
			
			}/*end if(trace_info_ptr_temp->packet_ID == packet_ID_temp)*/
		else
			{
			continue;
			}/*end else,and the corresponding if is if(trace_info_ptr_temp->packet_ID == packet_ID_temp), continue the loop*/
		
		}
	if(i == list_len)
		{
		op_prg_odb_print_major("****error,the packet is not found in ack_arr_actions****", OPC_NIL);
		}	
	
	FOUT;
}









/* 光包到达时需要完成两项工作，一，统计能耗和损耗，二，释放端口，如果该路由器连接IP核，则还需要释放用来存电建链包的队列*/
void optical_arr_actions(Packet *pkptr)
{
   
	int packet_ID_temp = -1;
	int list_len = -1;
	int i = -1;
	double opt_power_temp = 0;
	double opt_loss_temp = 0;
	double ring_on_time_temp = 0;
	Optical_Stat optical_stat_temp;
	Trace_Info *trace_info_ptr_temp;
	
	FIN(optical_arrival_actions(pkptr));
	
	op_pk_fd_get(pkptr, ID_NO, &packet_ID_temp);
	op_pk_fd_get(pkptr, OPT_POWER_FIELD, &opt_power_temp);
	op_pk_fd_get(pkptr, OPT_LOSS_FIELD , &opt_loss_temp);
	
	list_len = op_prg_list_size(lptr_pathsetup_info);
	
	for (i = 0; i < list_len; i++)
		{
		trace_info_ptr_temp = (Trace_Info*)op_prg_list_access(lptr_pathsetup_info, i);
		
		if (trace_info_ptr_temp->packet_ID == packet_ID_temp)
			{
			ring_on_time_temp = op_sim_time() - trace_info_ptr_temp->ring_start_on_time;
			optical_stat_temp = optical_statistics_get(trace_info_ptr_temp->port_in, trace_info_ptr_temp->port_out, ring_on_time_temp);
			opt_power_temp    = opt_power_temp + optical_stat_temp.power;
			opt_loss_temp     = opt_loss_temp  + optical_stat_temp.loss;
			
			op_pk_fd_set(pkptr, OPT_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, opt_power_temp, 20);
			op_pk_fd_set(pkptr, OPT_LOSS_FIELD , OPC_FIELD_TYPE_DOUBLE, opt_loss_temp , 10);
			op_pk_send_forced(pkptr, port_info[trace_info_ptr_temp->port_out].outstrm_index);

	        /*free the subq, only the router connecting IP core needs to do this action*/
			if (ip_num_of_this_router != NO_CORE_CONNECTTED)
				{
				switch (trace_info_ptr_temp->port_in)
				      {
				
//				       case 0 : SQ_Lock[local_node_address[0]][local_node_address[1]][0]=0;break;
				
//				       case 1 : SQ_Lock[local_node_address[0]][local_node_address[1]][1]=0;break;
				
//					   case 2 : SQ_Lock[local_node_address[0]][local_node_address[1]][2]=0;break;
					   
//					   case 3 : SQ_Lock[local_node_address[0]][local_node_address[1]][3]=0;break;
					   
					   case 4 : SQ_Lock[local_node_address[0]][local_node_address[1]][0]=0;break;
					   
				      }
				}/*end if (ip_num_of_this_router != NO_CORE_CONNECTTED)*/
			else
				{
				;/*do nothing in else*/
				}/*end of else, the corresponding if is if (ip_num_of_this_router != NO_CORE_CONNECTTED)*/
				

			/*free the output port*/
	        F_Port_Lock[local_node_address[0]][local_node_address[1]][trace_info_ptr_temp->port_out] = 0;
	        
	        /**/
	        trace_info_ptr_temp = (Trace_Info*)op_prg_list_remove(lptr_pathsetup_info, i);
			
			op_prg_mem_free(trace_info_ptr_temp);
			
			break;
			
			}/*end if (trace_info_ptr_temp->packet_ID == packet_ID_temp)*/
		else
			{
			continue;
			}/*end else, continue the loop*/
	    		
		}
	
	if(i == list_len)
		{
		op_prg_odb_print_major("****error,the packet is not found in optical_arr_actions****", OPC_NIL);
		}	
	
	FOUT;
	
}









static Optical_Stat optical_statistics_get(int port_in, int port_out, double ring_on_time)
{
	int port_to_port   = -1;
	int num_of_ring_on =  0;
	double power_temp  =  0;
	double loss_temp   =  0;
	Optical_Stat  optical_stat_temp;
	
	
	FIN (optical_statistics_get(port_in, port_out, ring_on_time));
	
	port_to_port = port_in * 10 + port_out;
	switch (port_to_port)
		{
		case port0_to_port0  :  op_prg_odb_print_major("error,the input_port should be different from the output_port,the error may happen in the routing function",OPC_NIL);
		                        break;
        case port0_to_port1  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port0_to_port1_length + 4 * bending_loss  
									        + 3 * crossing_loss + 2 * ring_through_loss + 1 * ring_drop_loss;/*损耗包括端口对之间的传播损耗，以及4个bending_loss*/
		                        break;                                                                      /*3个crossing_loss,2个ring_through_loss以及1个ring_drop_loss*/
        case port0_to_port2  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port0_to_port2_length + 3 * bending_loss  
									        + 8 * crossing_loss + 4 * ring_through_loss + 1 * ring_drop_loss;
		                        break;                 
        case port0_to_port3  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port0_to_port3_length + 2 * bending_loss  
									        + 7 * crossing_loss + 4 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port0_to_port4  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port0_to_port4_length + 3 * bending_loss 
									        + 2 * crossing_loss + 0 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port1_to_port0  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port1_to_port0_length + 5 * bending_loss  
									        + 5 * crossing_loss + 4 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port1_to_port1  :  op_prg_odb_print_major("error,the input_port should be different from the output_port,the error may happen in the routing function",OPC_NIL);
		                        break;
        case port1_to_port2  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port1_to_port2_length + 2 * bending_loss 
									        + 3 * crossing_loss + 1 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port1_to_port3  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port1_to_port3_length + 1 * bending_loss 
									        + 4 * crossing_loss + 2 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port1_to_port4  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port1_to_port4_length + 2 * bending_loss 
									        + 6 * crossing_loss + 4 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port2_to_port0  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port2_to_port0_length + 1 * bending_loss 
									        + 4 * crossing_loss + 2 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port2_to_port1  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port2_to_port1_length + 2 * bending_loss 
									        + 7 * crossing_loss + 4 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port2_to_port2  :  op_prg_odb_print_major("error,the input_port should be different from the output_port,the error may happen in the routing function",OPC_NIL);
		                        break;
        case port2_to_port3  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port2_to_port3_length + 0 * bending_loss 
									        + 3 * crossing_loss + 0 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port2_to_port4  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port2_to_port4_length + 1 * bending_loss 
									        + 8 * crossing_loss + 5 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port3_to_port0  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port3_to_port0_length + 2 * bending_loss 
									        + 4 * crossing_loss + 1 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port3_to_port1  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port3_to_port1_length + 3 * bending_loss 
									        + 9 * crossing_loss + 5 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port3_to_port2  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port3_to_port2_length + 2 * bending_loss 
							                + 2 * crossing_loss + 0 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port3_to_port3  :  op_prg_odb_print_major("error,the input_port should be different from the output_port,the error may happen in the routing function",OPC_NIL);
		                        break;
        case port3_to_port4  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port3_to_port4_length + 2 * bending_loss 
									        + 10 * crossing_loss + 6 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port4_to_port0  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port4_to_port0_length + 2 * bending_loss 
							                + 7 * crossing_loss + 4 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port4_to_port1  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port4_to_port1_length + 3 * bending_loss 
									        + 2 * crossing_loss + 0 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port4_to_port2  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port4_to_port2_length + 2 * bending_loss 
									        + 10 * crossing_loss + 6 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port4_to_port3  :  num_of_ring_on = 1;
		                        loss_temp = waveguide_propagation_loss * port4_to_port3_length + 1 * bending_loss 
									        + 8 * crossing_loss + 5 * ring_through_loss + 1 * ring_drop_loss;
		                        break;
        case port4_to_port4  :  op_prg_odb_print_major("error,the input_port should be different from the output_port,the error may happen in the routing function",OPC_NIL);
		                        break;
		default              :  op_prg_odb_print_major("error,the input_port and the output_port are out_of_range",OPC_NIL);
		}
	
	/*能量包括从off_to_on，以及处于on时，和on_to_off消耗的能量*/
	power_temp = num_of_ring_on * (ring_dynamic_power * ring_switch_time + ring_static_power * ring_on_time/1000000000 + ring_dynamic_power * ring_switch_time);

//	printf("ring_on_time=%f\n", ring_on_time/1000000000);
	
	if ((ip_num_of_this_router != NO_CORE_CONNECTTED) && (port_out == port_num - 1))/*判断下一跳是不是IP核，如果是，则需要统计连接IP核的那根波导的损耗*/
		loss_temp  = loss_temp + port_info[port_in].olink_length * waveguide_propagation_loss + port_info[port_in].olink_bending_loss 
	                           + port_info[port_in].olink_crossing_loss + port_info[port_out].olink_length * waveguide_propagation_loss 
		                       + port_info[port_out].olink_bending_loss + port_info[port_out].olink_crossing_loss;
	/*end if((ip_num_of_this_router != NO_CORE_CONNECTTED)&& (port_out == 4)),这里的损耗值为输入和输出波导以及光开关内部的损耗*/
	else
		{
		loss_temp = loss_temp + port_info[port_in].olink_length * waveguide_propagation_loss + port_info[port_in].olink_bending_loss 
	                          + port_info[port_in].olink_crossing_loss;
		}/*end else,只统计输入波导的损耗和光开关内部的损耗*/
		
	
	optical_stat_temp.power = power_temp;
	optical_stat_temp.loss  = loss_temp ;
	
	FRET(optical_stat_temp);
}








/* 轮询函数，对本地路由器的各个端口进行轮询，将具有相同输出端口分组的输入端口和输出端口放入对应输出端口的候选列表中*/
void rolling_check_func()
{
	Packet *pkptr;
	candidate_Info *candidate_Info_ptr;
	int port_out_temp = -1;
	int i = -1;
	
	
    FIN(rolling_check_func());
	
	for (i = 0; i < port_num; i++)
	{
		if (op_subq_empty(i) == OPC_TRUE)
			{
		    continue;
			}/*end of if*/		
        else 
		   {
		   if (i == port_num - 1 && SQ_Lock[local_node_address[0]][local_node_address[1]][0] == 1)
			   {
			   continue;
			   }/*end of if*/
		   
		   pkptr = op_subq_pk_access(i, OPC_QPOS_HEAD);	/* 获得第i个子队列队首的分组*/		
		   port_out_temp = mesh_routing(pkptr);         /* 对该分组进行路由，获得分组在本地路由器的输出端口*/
			
		   /* 判断分组输出端口是否已被锁，若没有，则将分组的输入、输出端口放入对应输出端口的候选发送队列中*/
		   if(F_Port_Lock[local_node_address[0]][local_node_address[1]][port_out_temp]!=1)
			   {			  
				candidate_Info_ptr           = (candidate_Info *)op_prg_mem_alloc (sizeof (candidate_Info));
				candidate_Info_ptr->subq_in  = i;
				candidate_Info_ptr->port_out = port_out_temp;
				op_prg_list_insert(lptr_candidate_info[port_out_temp], candidate_Info_ptr, OPC_LISTPOS_TAIL);
			   }/*end of if*/
		    }/*end of else*/
     }/*end of for*/
	FOUT;
}







/*mesh中的路由算法*/
static int mesh_routing(Packet *pkptr)
	{
	int port = -1;
	int D_X_temp = -1, D_Y_temp = -1;

	FIN (mesh_routing(pkptr));
	
	op_pk_fd_get(pkptr, DEST_ADDR_FIELD_X , &D_X_temp);
	op_pk_fd_get(pkptr, DEST_ADDR_FIELD_Y , &D_Y_temp);

	if (D_X_temp != local_node_address[0])
		{
		if (D_X_temp > local_node_address[0])
			{
			port = 2;
			}
		else
			{
			port = 0;	
			}
		}/*end of if*/
	else if (D_Y_temp != local_node_address[1])
		{
		if (D_Y_temp > local_node_address[1])
			{
			port = 1;
			}
		else
			{
			port = 3;	
			}
		}/*end of else if*/
	else
		{
		port = 4;
		}/*end of else*/
	    
	FRET(port);/*返回路由后得到的输出端口*/
	}






/*轮询各端口候选发送队列，并选出一个分组发送*/
void candidate_list_rolling()
	{
	int i = -1;
	int list_len = -1;
	
	FIN(candidate_list_rolling());
   	
	for (i = 0; i < port_num; i++)
		{		
		list_len = op_prg_list_size(lptr_candidate_info[i]);/*获取第i个端口的候选队列的长度*/
		
		/*判断队列时候为空，若为空，则跳出本次轮询，否则从该端口的候选队列中通过调用函数选出一个分组发送*/
		if (list_len == 0)
			{
			continue;
			}/*end of if*/		
		else
			{			
			candidate_chosen_to_send(list_len, i);/*调用选分组发送的函数*/
			}
		}/*end of for*/
	
	 FOUT;
	}






/*在候选发送分组队列中选取分组进行发送，并将电建链分组的能耗和跳数进行统计*/
void candidate_chosen_to_send(int list_len, int i)
	{
	int j = -1;
	int num_chosen = -1;
	candidate_Info *candidate_Info_ptr;
	Distribution *num_distribut;
	
	Packet *pkptr;
	int packet_ID_temp = 0;
	Trace_Info *trace_info_ptr_temp;
	
	double elec_power_field_temp;
	double elec_power_this_jump;
	int hop_field_temp;

	FIN(candidate_chosen_to_send(list_len,i));
	
    /*在1到list_len中随机选取一个数，作为发送分组的位置*/	
	num_distribut = op_dist_load("uniform_int", 1, list_len);
	   num_chosen = op_dist_outcome(num_distribut);
	
	/*从长为list_len的队列中取出已经选定位置的分组，统计能耗和跳数，并发送*/
    for (j = 0; j < list_len; j++)
		{
	  
	    /*判断j是否为选定的分组位置，若不是，则跳出本次循环*/
	    if (j != num_chosen-1)
			{
		    continue;
		    }/*end of if*/
	    else
			{
		    candidate_Info_ptr = (candidate_Info *)op_prg_list_access(lptr_candidate_info[i], j);/*从端口候选发送列表中得到第j个候选分组信息*/
		    pkptr = op_subq_pk_remove (candidate_Info_ptr->subq_in, OPC_QPOS_HEAD);              /*从候选队列中获取分组*/		
		
		    trace_info_ptr_temp = (Trace_Info*)op_prg_mem_alloc (sizeof (Trace_Info));           /*创建存放建链包链路信息的表项*/
		
		    /* 记录发送的建链包的路径，包括分组的ID，输入端口，输出端口，并将微环处于on的初始时间设为0 */	
		    op_pk_fd_get(pkptr, ID_NO, &packet_ID_temp);			
		    trace_info_ptr_temp->packet_ID           = packet_ID_temp;  		
		    trace_info_ptr_temp->port_in             = candidate_Info_ptr->subq_in;
		    trace_info_ptr_temp->port_out            = candidate_Info_ptr->port_out;
		    trace_info_ptr_temp->ring_start_on_time  = 0;
		
		    op_prg_list_insert(lptr_pathsetup_info, trace_info_ptr_temp, OPC_LISTPOS_TAIL);        /*将建链包的路径信息存入建链信息列表*/
		
		    /* 将在本路由器和上一条链路的能耗加入分组的能耗域*/
		    op_pk_fd_get(pkptr, ELEC_POWER_FIELD, &elec_power_field_temp);
		    elec_power_this_jump  = electrical_power_calculate(candidate_Info_ptr->subq_in, candidate_Info_ptr->port_out, PATHSETUP_FLAG);
		    elec_power_field_temp = elec_power_field_temp + elec_power_this_jump;
		    op_pk_fd_set(pkptr, ELEC_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, elec_power_field_temp, 20);
		
		    /* 将跳数更新*/
		    op_pk_fd_get(pkptr, HOP_FIELD, &hop_field_temp);
		    hop_field_temp = hop_field_temp + 1;
			if (candidate_Info_ptr->port_out == port_num - 1)
				{
				hop_field_temp = hop_field_temp + 1;
				}
		    op_pk_fd_set(pkptr, HOP_FIELD, OPC_FIELD_TYPE_INTEGER, hop_field_temp, 5);	
			
			F_Port_Lock[local_node_address[0]][local_node_address[1]][candidate_Info_ptr->port_out] = 1;/*将建链包的输出端口锁住*/
		
		    op_pk_send_forced(pkptr, port_info[candidate_Info_ptr->port_out].outstrm_index);            /*将建链包发送到对应端口的输出流*/
		
		   
		
		    /* 如果建链分组是从IP核发来的，就将路由器中的第4个子队列锁住*/
		    if (candidate_Info_ptr->subq_in == port_num - 1)
				{
			    SQ_Lock[local_node_address[0]][local_node_address[1]][0] = 1;
				}/*end of if*/
			}/*end of else*/
		}/*end of for*/ 
	
	 FOUT;
	
	}






static double electrical_power_calculate(int port_in, int port_out, int flag)
	{
	double electrical_power;
	int packet_length;
	double link_length_temp;
	
	
	FIN(electrical_power_calculate(port_in, port_out, flag));
	
	/* 电分组为建链包的计算情况*/
	if (flag == 0)
		{
		packet_length     = path_setup_length;
		link_length_temp  = port_info[port_in].elink_length;			
		electrical_power  = router_power*packet_length + wire_power*link_length_temp*packet_length;
		
		/* 判断下一跳是否达到IP核，若是，则将下一跳链路的能耗加入*/
		if (port_out == port_num - 1)
			{
		    link_length_temp = port_info[port_out].elink_length;
		    electrical_power = electrical_power + wire_power*link_length_temp*packet_length;
			}/*end of port_out==4*/		
		}/*end of 'flag==0'*/
	
	/* 电分组为ACK的计算情况*/
	else if (flag == 2)
		{
		packet_length     = ack_length;
		link_length_temp  = port_info[port_in].elink_length;
		electrical_power  = crossbar_power*packet_length + wire_power*link_length_temp*packet_length;
		
		/* 判断下一跳是否达到IP核，若是，则将下一跳链路的能耗加入*/
		if (port_out == port_num - 1)
			{
		    link_length_temp = port_info[port_out].elink_length;
		    electrical_power = electrical_power + wire_power*link_length_temp*packet_length;
			}/*end of port_out==4*/			
		}/*end of 'flag==2'*/
	
	FRET(electrical_power);
	
	}








/*将端口的候选发送分组队列清空*/
void list_candidate_empty_set()
	{
	int i=-1;
	
	FIN(list_candidate_empty_set());
	
	for (i = 0; i < port_num; i++)
		{
		op_prg_list_free(lptr_candidate_info[i]);/*释放第i个端口的候选列表*/
		}/*end of for*/
	
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
	void router_mesh_switch (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_router_mesh_switch_init (int * init_block_ptr);
	void _op_router_mesh_switch_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_router_mesh_switch_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_router_mesh_switch_alloc (VosT_Obtype, int);
	void _op_router_mesh_switch_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
router_mesh_switch (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (router_mesh_switch ());

		{
		/* Temporary Variables */
		/////////////////////////////////////////////
		/**/
		Objid rx_objid;
		Objid rx_strm_objid;
		Objid tx_objid;
		Objid tx_strm_objid;	
		Objid link_objid;
		Objid remote_node_objid;
		
		/**/
		int remote_node_address[3];
		int rx_strm_index;
		int	tx_strm_index;
		int port;
		int i,j,k;
		//2015.5.9 tw
		int uplimit_of_for;
		/**/
		int instrm;
		Packet * pkptr;
		/* End of Temporary Variables */


		FSM_ENTER ("router_mesh_switch")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (init) enter executives **/
			FSM_STATE_ENTER_FORCED_NOLABEL (0, "init", "router_mesh_switch [init enter execs]")
				FSM_PROFILE_SECTION_IN ("router_mesh_switch [init enter execs]", state0_enter_exec)
				{
				
				surr_mod_objid = op_id_self();/*获得该模块的ID*/
				
				surr_node_objid = op_id_parent(surr_mod_objid);/*获得该模块所属节点的ID*/
				
				/*获得节点属性*/
				op_ima_obj_attr_get (surr_node_objid, "Node_Address_X", &local_node_address[0]);
				op_ima_obj_attr_get (surr_node_objid, "Node_Address_Y", &local_node_address[1]);
				op_ima_obj_attr_get (surr_node_objid, "Node_Address_Z", &local_node_address[2]);
				op_ima_obj_attr_get (surr_node_objid, "Ip_Num_Of_This_Router", &ip_num_of_this_router);
				op_ima_obj_attr_get (surr_node_objid, "Router_Power(J/bit)", &router_power);
				op_ima_obj_attr_get (surr_node_objid, "Crossbar_Power(J/bit)", &crossbar_power);
				op_ima_obj_attr_get (surr_node_objid, "Ring_Switch_Time(s)", &ring_switch_time);
				op_ima_obj_attr_get (surr_node_objid, "Ring_Static_Power(W)", &ring_static_power);
				op_ima_obj_attr_get (surr_node_objid, "Ring_Dynamic_Power(W)", &ring_dynamic_power);
				op_ima_obj_attr_get (surr_node_objid, "Ring_Drop_Loss(dB)", &ring_drop_loss);
				op_ima_obj_attr_get (surr_node_objid, "Ring_Through_Loss(dB)", &ring_through_loss);
				op_ima_obj_attr_get (surr_node_objid, "Bending_Loss(dB)", &bending_loss);
				op_ima_obj_attr_get (surr_node_objid, "Crossing_Loss(dB)", &crossing_loss);
				
				/*获取仿真属性*/
				op_ima_sim_attr_get (OPC_IMA_DOUBLE, "Roll Around Period(ns)", &inter_check_period);
				op_ima_sim_attr_get (OPC_IMA_DOUBLE, "Transmission Bandwidth (Gbps)", &transmission_bandwidth);
				op_ima_sim_attr_get (OPC_IMA_DOUBLE, "Optical Fixed Packet Length(bit)", &OP_length);
				op_ima_sim_attr_get (OPC_IMA_DOUBLE, "Path Setup Length(bit)", &path_setup_length);
				op_ima_sim_attr_get (OPC_IMA_DOUBLE, "Ack Length(bit)", &ack_length);
				op_ima_sim_attr_get (OPC_IMA_DOUBLE, "Number Of Out Port", &port_num);
				op_ima_sim_attr_get (OPC_IMA_DOUBLE, "Dim Of Mesh", &dim_of_mesh);
				
				port_info = (Port_Info *) op_prg_mem_alloc(port_num*sizeof(Port_Info));/*给每个端口分配一定大小的结构体，用于保存相关信息*/
				
				/*初始化各端口的参数*/
				for (i = 0; i < port_num; i++)
					{
					port_info[i].outstrm_index       = -1;
					port_info[i].instrm_index        = -1;
					port_info[i].elink_length        = 0;
					port_info[i].olink_length        = 0;
					port_info[i].olink_bending_loss  = 0;
					port_info[i].olink_crossing_loss = 0;
					}
				//2015.5.9 tw 修改之前的链路初始化的可恢复错误
				if(local_node_address[0] == 0 || local_node_address[0] == dim_of_mesh - 1 ||
				   local_node_address[1] == 0 || local_node_address[1] == dim_of_mesh - 1)
					{
					if((local_node_address[0] == 0 && local_node_address[1] == 0) ||
					   (local_node_address[0] == 0 && local_node_address[1] == dim_of_mesh - 1) ||
					   (local_node_address[0] == dim_of_mesh - 1 && local_node_address[1] == 0) ||
					   (local_node_address[0] == dim_of_mesh - 1 && local_node_address[1] == dim_of_mesh - 1))
						{
						uplimit_of_for = port_num -2;
						}
					else {
						uplimit_of_for = port_num -1;
						}
					}
				else
					{
					uplimit_of_for = port_num;
					}
				//end of revise
				
				for (i = 0; i < uplimit_of_for; i++)
					{
					
				    /*获得与该模块相连的第i个接收机的ID*/
					rx_objid      = op_topo_assoc(surr_mod_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_PTRX, i);
					rx_strm_objid = op_topo_connect(rx_objid, surr_mod_objid, OPC_OBJTYPE_STRM, 0);
					op_ima_obj_attr_get(rx_strm_objid, "dest stream", &rx_strm_index);
					
					/* 获得链路ID*/
					if (op_topo_assoc (rx_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_LKDUP, 0) == OPC_OBJID_INVALID)
						{
						continue;
						}/*end of if*/
					else
						{
						link_objid = op_topo_assoc (rx_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_LKDUP, 0);
						}/*end of else*/
					
					/* 获取链路属性*/
					op_ima_obj_attr_get(link_objid, "Wire_Length(m)", &wire_length);
					op_ima_obj_attr_get(link_objid, "Wire_Propagation_Rate(m/s)", &wire_propagation_rate);
					op_ima_obj_attr_get(link_objid, "Wire_Power(J/m/bit)", &wire_power);
					op_ima_obj_attr_get(link_objid, "Waveguide_Length(m)", &waveguide_length);
					op_ima_obj_attr_get(link_objid, "Waveguide_Propagation_Rate(m/s)", &waveguide_propagation_rate);
				    op_ima_obj_attr_get(link_objid, "Waveguide_Propagation_Loss(dB/m)", &waveguide_propagation_loss);
					op_ima_obj_attr_get(link_objid, "Waveguide_Bending_Loss(dB)", &waveguide_bending_loss);
					op_ima_obj_attr_get(link_objid, "Waveguide_Crossing_Loss(dB)", &waveguide_crossing_loss);
					op_ima_obj_attr_get(link_objid, "data rate", &data_rate);
					
					/* 通过链路ID获得与之相连的发射机ID，进而获得与本节点相连的相邻节点中发射机ID*/
					tx_objid = op_topo_assoc(link_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_PTTX, 0);
					if (op_topo_parent (tx_objid) == surr_node_objid)
						{
					    tx_objid = op_topo_assoc(link_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_PTTX, 1);
						}/*end of if*/
					
					/* 通过发射机ID确定与该节点相连的相邻节点ID，并获取节点地址*/
				    remote_node_objid = op_topo_parent(tx_objid);
					op_ima_obj_attr_get(remote_node_objid, "Node_Address_X", &remote_node_address[0]);
					op_ima_obj_attr_get(remote_node_objid, "Node_Address_Y", &remote_node_address[1]);	
					op_ima_obj_attr_get(remote_node_objid, "Node_Address_Z", &remote_node_address[2]);
					
				
				    /* 端口分配 */
				   if ((remote_node_address[0] == local_node_address[0]) && (remote_node_address[1]==local_node_address[1])
					   &&(remote_node_address[2] == local_node_address[2]))
					   {
					   port = 4;
					   }/*end of if*/ 
				   else if ((remote_node_address[0] != local_node_address[0]) && (remote_node_address[1] ==local_node_address[1])
					   &&(remote_node_address[2] == local_node_address[2]))
					   {
				         if (remote_node_address[0] > local_node_address[0])
							 {
							 port = 2;
							 }
						 else
							 {
							 port = 0;
							 }
				   
				        }/*end of else if*/
				   else if ((remote_node_address[0] == local_node_address[0]) && (remote_node_address[1] != local_node_address[1])
					   &&(remote_node_address[2] == local_node_address[2]))
					   {
					    if (remote_node_address[1] > local_node_address[1])
							{
							 port = 1;
							 }
						 else
							 {
							 port = 3;
							 }
					   }/*end of else if*/
				
					
				 
					 
				    /* 保存与该端口相关的各种信息*/	
					port_info[port].instrm_index        = rx_strm_index;
					port_info[port].elink_length        = wire_length;
					port_info[port].olink_length        = waveguide_length;
					port_info[port].olink_bending_loss  = waveguide_bending_loss;
					port_info[port].olink_crossing_loss = waveguide_crossing_loss;
					
					
					/* 获取该路由器中特定端口的发射机的ID*/
					tx_objid = op_topo_assoc(link_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_PTTX, 0);
					if (op_topo_parent(tx_objid) != surr_node_objid)
						{
					    tx_objid = op_topo_assoc(link_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_PTTX, 1);
						}/*end of if*/
					
					/*获得特定端口的输出流流号并储存*/
					tx_strm_objid = op_topo_connect(surr_mod_objid,tx_objid,OPC_OBJTYPE_STRM,0);/*获得连接发射机的模块的ID */
				    op_ima_obj_attr_get(tx_strm_objid, "src stream", &tx_strm_index);/*并取其流号*/
					port_info[port].outstrm_index = tx_strm_index;/*储存端口的输出流流号*/	
				   }/*end of for*/
				
				
				/*将各端口的锁置为0,即处于解锁状态*/
				for (i = 0; i < dim_of_Xrouter; i++)
					{
					for (j = 0; j < dim_of_Yrouter; j++)
						{
						for (k = 0; k < port_out_num; k++)
							{
							F_Port_Lock[i][j][k] = 0;
							}
						}
					}
				
				/* 初始化各队列的锁为0，即处于解锁状态*/
				for (i = 0; i < dim_of_Xrouter; i++)
					{
					for (j = 0; j < dim_of_Yrouter; j++)
						{
						for (k = 0; k < max_IP_num_of_router; k++)
							{
						    SQ_Lock[i][j][k] = 0;
							}
						}
					}
				
				/* 在每个端口都分配一个存储轮询时产生候选分组的队列 */
				for (i=0; i<port_num; i++)
					{
					lptr_candidate_info[i] = op_prg_list_create();
					}
				
				lptr_pathsetup_info = op_prg_list_create();/* 产生建链信息列表*/
				
				op_intrpt_schedule_self(op_sim_time(), ROLLING_CHECK);/* 开启轮询中断*/
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** state (init) exit executives **/
			FSM_STATE_EXIT_FORCED (0, "init", "router_mesh_switch [init exit execs]")


			/** state (init) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "init", "idle", "tr_1", "router_mesh_switch [init -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (idle) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "idle", state1_enter_exec, "router_mesh_switch [idle enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"router_mesh_switch")


			/** state (idle) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "idle", "router_mesh_switch [idle exit execs]")


			/** state (idle) transition processing **/
			FSM_PROFILE_SECTION_IN ("router_mesh_switch [idle trans conditions]", state1_trans_conds)
			FSM_INIT_COND (ROLLING)
			FSM_TEST_COND (ARRIVAL_ACT)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("idle")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 5, state5_enter_exec, ;, "ROLLING", "", "idle", "rolling_check", "tr_0", "router_mesh_switch [idle -> rolling_check : ROLLING / ]")
				FSM_CASE_TRANSIT (1, 2, state2_enter_exec, ;, "ARRIVAL_ACT", "", "idle", "arrival_act", "tr_2", "router_mesh_switch [idle -> arrival_act : ARRIVAL_ACT / ]")
				FSM_CASE_TRANSIT (2, 1, state1_enter_exec, ;, "default", "", "idle", "idle", "tr_8", "router_mesh_switch [idle -> idle : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (arrival_act) enter executives **/
			FSM_STATE_ENTER_FORCED (2, "arrival_act", state2_enter_exec, "router_mesh_switch [arrival_act enter execs]")
				FSM_PROFILE_SECTION_IN ("router_mesh_switch [arrival_act enter execs]", state2_enter_exec)
				{
				
				instrm = op_intrpt_strm();/*获得中断流*/
				
				pkptr = op_pk_get(instrm);/*从中断流中获取分组*/
				
				
				/*电分组到达*/
				if (op_pk_encap_flag_is_set(pkptr, PATHSETUP_FLAG))
					{
					electronic_arr_actions(pkptr, instrm);
					}
				
				/*ACK到达*/
				else if (op_pk_encap_flag_is_set(pkptr, ACK_FLAG))
					{
				    ack_arr_actions(pkptr);
					}
				
				/*光分组到达*/
				else if(op_pk_encap_flag_is_set(pkptr, OPTICAL_FLAG))
					{
					optical_arr_actions(pkptr);
					}
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** state (arrival_act) exit executives **/
			FSM_STATE_EXIT_FORCED (2, "arrival_act", "router_mesh_switch [arrival_act exit execs]")


			/** state (arrival_act) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "arrival_act", "idle", "tr_3", "router_mesh_switch [arrival_act -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (list_empty) enter executives **/
			FSM_STATE_ENTER_FORCED (3, "list_empty", state3_enter_exec, "router_mesh_switch [list_empty enter execs]")
				FSM_PROFILE_SECTION_IN ("router_mesh_switch [list_empty enter execs]", state3_enter_exec)
				{
				list_candidate_empty_set();
				
				op_intrpt_schedule_self(op_sim_time() + inter_check_period, ROLLING_CHECK);
				}
				FSM_PROFILE_SECTION_OUT (state3_enter_exec)

			/** state (list_empty) exit executives **/
			FSM_STATE_EXIT_FORCED (3, "list_empty", "router_mesh_switch [list_empty exit execs]")


			/** state (list_empty) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "list_empty", "idle", "tr_11", "router_mesh_switch [list_empty -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (list_rolling) enter executives **/
			FSM_STATE_ENTER_FORCED (4, "list_rolling", state4_enter_exec, "router_mesh_switch [list_rolling enter execs]")
				FSM_PROFILE_SECTION_IN ("router_mesh_switch [list_rolling enter execs]", state4_enter_exec)
				{
				candidate_list_rolling();
				}
				FSM_PROFILE_SECTION_OUT (state4_enter_exec)

			/** state (list_rolling) exit executives **/
			FSM_STATE_EXIT_FORCED (4, "list_rolling", "router_mesh_switch [list_rolling exit execs]")


			/** state (list_rolling) transition processing **/
			FSM_TRANSIT_FORCE (3, state3_enter_exec, ;, "default", "", "list_rolling", "list_empty", "tr_10", "router_mesh_switch [list_rolling -> list_empty : default / ]")
				/*---------------------------------------------------------*/



			/** state (rolling_check) enter executives **/
			FSM_STATE_ENTER_FORCED (5, "rolling_check", state5_enter_exec, "router_mesh_switch [rolling_check enter execs]")
				FSM_PROFILE_SECTION_IN ("router_mesh_switch [rolling_check enter execs]", state5_enter_exec)
				{
				if (op_q_empty() == OPC_FALSE)
					{
					
					rolling_check_func(); 
						
					}
				
				
				}
				FSM_PROFILE_SECTION_OUT (state5_enter_exec)

			/** state (rolling_check) exit executives **/
			FSM_STATE_EXIT_FORCED (5, "rolling_check", "router_mesh_switch [rolling_check exit execs]")


			/** state (rolling_check) transition processing **/
			FSM_TRANSIT_FORCE (4, state4_enter_exec, ;, "default", "", "rolling_check", "list_rolling", "tr_9", "router_mesh_switch [rolling_check -> list_rolling : default / ]")
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"router_mesh_switch")
		}
	}




void
_op_router_mesh_switch_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
	/* No Diagnostic Block */
	}




void
_op_router_mesh_switch_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_router_mesh_switch_terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_router_mesh_switch_svar function. */
#undef surr_mod_objid
#undef surr_node_objid
#undef local_node_address
#undef lptr_pathsetup_info
#undef port_info
#undef inter_check_period
#undef transmission_bandwidth
#undef OP_length
#undef lptr_candidate_info
#undef port_num
#undef ring_drop_loss
#undef ring_dynamic_power
#undef ring_static_power
#undef ring_through_loss
#undef router_power
#undef wire_length
#undef wire_propagation_rate
#undef wire_power
#undef waveguide_length
#undef waveguide_propagation_rate
#undef waveguide_propagation_loss
#undef waveguide_bending_loss
#undef waveguide_crossing_loss
#undef data_rate
#undef path_setup_length
#undef ack_length
#undef dim_of_mesh
#undef crossbar_power
#undef ip_num_of_this_router
#undef ring_switch_time
#undef bending_loss
#undef crossing_loss

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_router_mesh_switch_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_router_mesh_switch_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (router_mesh_switch)",
		sizeof (router_mesh_switch_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_router_mesh_switch_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	router_mesh_switch_state * ptr;
	FIN_MT (_op_router_mesh_switch_alloc (obtype))

	ptr = (router_mesh_switch_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "router_mesh_switch [init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_router_mesh_switch_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	router_mesh_switch_state		*prs_ptr;

	FIN_MT (_op_router_mesh_switch_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (router_mesh_switch_state *)gen_ptr;

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
	if (strcmp ("local_node_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->local_node_address);
		FOUT
		}
	if (strcmp ("lptr_pathsetup_info" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->lptr_pathsetup_info);
		FOUT
		}
	if (strcmp ("port_info" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->port_info);
		FOUT
		}
	if (strcmp ("inter_check_period" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->inter_check_period);
		FOUT
		}
	if (strcmp ("transmission_bandwidth" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->transmission_bandwidth);
		FOUT
		}
	if (strcmp ("OP_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->OP_length);
		FOUT
		}
	if (strcmp ("lptr_candidate_info" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->lptr_candidate_info);
		FOUT
		}
	if (strcmp ("port_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->port_num);
		FOUT
		}
	if (strcmp ("ring_drop_loss" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ring_drop_loss);
		FOUT
		}
	if (strcmp ("ring_dynamic_power" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ring_dynamic_power);
		FOUT
		}
	if (strcmp ("ring_static_power" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ring_static_power);
		FOUT
		}
	if (strcmp ("ring_through_loss" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ring_through_loss);
		FOUT
		}
	if (strcmp ("router_power" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->router_power);
		FOUT
		}
	if (strcmp ("wire_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->wire_length);
		FOUT
		}
	if (strcmp ("wire_propagation_rate" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->wire_propagation_rate);
		FOUT
		}
	if (strcmp ("wire_power" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->wire_power);
		FOUT
		}
	if (strcmp ("waveguide_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->waveguide_length);
		FOUT
		}
	if (strcmp ("waveguide_propagation_rate" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->waveguide_propagation_rate);
		FOUT
		}
	if (strcmp ("waveguide_propagation_loss" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->waveguide_propagation_loss);
		FOUT
		}
	if (strcmp ("waveguide_bending_loss" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->waveguide_bending_loss);
		FOUT
		}
	if (strcmp ("waveguide_crossing_loss" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->waveguide_crossing_loss);
		FOUT
		}
	if (strcmp ("data_rate" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_rate);
		FOUT
		}
	if (strcmp ("path_setup_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->path_setup_length);
		FOUT
		}
	if (strcmp ("ack_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ack_length);
		FOUT
		}
	if (strcmp ("dim_of_mesh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->dim_of_mesh);
		FOUT
		}
	if (strcmp ("crossbar_power" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->crossbar_power);
		FOUT
		}
	if (strcmp ("ip_num_of_this_router" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ip_num_of_this_router);
		FOUT
		}
	if (strcmp ("ring_switch_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ring_switch_time);
		FOUT
		}
	if (strcmp ("bending_loss" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->bending_loss);
		FOUT
		}
	if (strcmp ("crossing_loss" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->crossing_loss);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

