/* Process model C form file: Ip_Source_mesh.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char Ip_Source_mesh_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A modeler 7 57271079 57271079 1 CN13549845642 Administrator 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1bcc 1                                                                                                                                                                                                                                                                                                                                                                                              ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */


#include "opnet.h"
#include "math.h"
#include <stdio.h>


/*定义中断码*/
#define	 START			0
#define	 GENERATE			1



/*定义状态转移条件*/
#define     SSC_START             (op_intrpt_code() == START)
#define     END_SIM               (op_intrpt_type() == OPC_INTRPT_ENDSIM)
#define		PACKET_GENERATE	      (op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code()== GENERATE)



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


/* 定义时间和空间的流量模型 */

#define   UNIFORM_TRAFFIC           1
#define   MATRIX_REVERSE            2
#define   HOTPOT_TRAFFIC_POINT      3
#define   HOTPOT_TRAFFIC_AREA       4
#define   LOCAL_TRAFFIC             5
#define   BIT_REVERSE               6
#define   POINT_TO_POINT_TRAFFIC    7










/*定义不同分组的标识*/
#define PATHSETUP_FLAG        0
#define OPTICAL_FLAG          1
#define ACK_FLAG              2



/*定义统计变量*/
int id_num_global = 0;
int gen_pkts=0;
int DestRcv[8][8]={{0},{0},{0}};//统计每个目的节点接收到包的个数
int RecToRcv[8][8][8][8]={0};  //统计目的节点从每个源节点接收到的包的个数




/*定义产生目的地址的结构体*/
typedef struct 
	{ int x;
	  int y;
	  } T_ADDRESS;



/*函数声明*/


static T_ADDRESS uniform_dist_gen(T_ADDRESS address_gen, int dim);

static T_ADDRESS matrix_reverse(T_ADDRESS address_gen, int dim);

static T_ADDRESS hotpot_traffic_type_point(T_ADDRESS address_gen, int dim);

static T_ADDRESS hotpot_traffic_type_area(T_ADDRESS address_gen, int dim);

static T_ADDRESS local_traffic(T_ADDRESS address_gen, int dim);

static T_ADDRESS bit_reverse(T_ADDRESS address_gen, int dim);



static void ss_packet_generate();

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
	int	                    		local_node_address[3]                           ;
	Objid	                  		own_objid                                       ;
	Objid	                  		node_objid                                      ;
	Evhandle	               		next_pk_evh                                     ;
	double	                 		next_intarr_time                                ;
	double	                 		mean_pk_arrival_time                            ;
	int	                    		send_flag                                       ;
	int	                    		END_PER                                         ;
	double	                 		OP_length                                       ;
	double	                 		Offered_load                                    ;
	double	                 		transmission_bandwidth                          ;
	double	                 		Dim                                             ;
	int	                    		traffic_module                                  ;
	double	                 		ack_length                                      ;
	double	                 		path_setup_length                               ;
	} Ip_Source_mesh_state;

#define local_node_address      		op_sv_ptr->local_node_address
#define own_objid               		op_sv_ptr->own_objid
#define node_objid              		op_sv_ptr->node_objid
#define next_pk_evh             		op_sv_ptr->next_pk_evh
#define next_intarr_time        		op_sv_ptr->next_intarr_time
#define mean_pk_arrival_time    		op_sv_ptr->mean_pk_arrival_time
#define send_flag               		op_sv_ptr->send_flag
#define END_PER                 		op_sv_ptr->END_PER
#define OP_length               		op_sv_ptr->OP_length
#define Offered_load            		op_sv_ptr->Offered_load
#define transmission_bandwidth  		op_sv_ptr->transmission_bandwidth
#define Dim                     		op_sv_ptr->Dim
#define traffic_module          		op_sv_ptr->traffic_module
#define ack_length              		op_sv_ptr->ack_length
#define path_setup_length       		op_sv_ptr->path_setup_length

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	Ip_Source_mesh_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((Ip_Source_mesh_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif


/* 定义均匀流量模型 */
 static T_ADDRESS uniform_dist_gen(T_ADDRESS address_gen, int dim)
	   
	 
	{ 
	    T_ADDRESS address_source_temp;
	    T_ADDRESS address_dest_temp;
		
	    int dim_temp;
		Distribution* uniform_dist;
		dim_temp = dim;
		address_source_temp.x = address_gen.x;		
		address_source_temp.y = address_gen.y;
		uniform_dist = op_dist_load("uniform_int",0,dim_temp-1);//加载分布函数
		do
			{
			  address_dest_temp.x = op_dist_outcome(uniform_dist);
			  address_dest_temp.y = op_dist_outcome(uniform_dist);
			  
			} while(address_dest_temp.x == address_source_temp.x && address_dest_temp.y == address_source_temp.y);
		
		op_dist_unload(uniform_dist);//释放内存
	
		return(address_dest_temp);
		
	 }
	 
	 
	 
	 
	 	 
/* 定义矩阵转置流量模型 */

static T_ADDRESS matrix_reverse(T_ADDRESS address_gen, int dim)
	  

	{ 
	    T_ADDRESS address_source_temp;
	    T_ADDRESS address_dest_temp;
		
		int dim_temp;
		dim_temp = dim;
		address_source_temp.x = address_gen.x;		
		address_source_temp.y = address_gen.y;
		address_dest_temp.x = dim_temp-address_source_temp.x-1;
		address_dest_temp.y = dim_temp-address_source_temp.y-1;

		return(address_dest_temp);	
	}
				
	
/* 定义热点流量模型,规定某个固定节点为热点 */
	 
 static T_ADDRESS hotpot_traffic_type_point(T_ADDRESS address_gen, int dim )
	 { 
	    T_ADDRESS address_source_temp;
	    T_ADDRESS address_dest_temp;
		int dim_temp;
		int state = -1;
		int ratio;
		
		Distribution* uniform_dist_percent;
		Distribution* uniform_dist_address;
		
		dim_temp = dim ;
		address_source_temp.x = address_gen.x ;
		address_source_temp.y = address_gen.y ;
		
		uniform_dist_percent = op_dist_load( "uniform_int" ,0,99);
		ratio = op_dist_outcome( uniform_dist_percent ) ;
		
		if( ratio < 10 )  state = 2 ;
		else             state = 1 ;
		
		
		
		uniform_dist_address = op_dist_load ( "uniform_int" , 0, dim_temp-1);
		switch (state)
			{
			 case 1 :
		         do
					 { 
					 
			//		  printf(" state 1 : global!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
					  address_dest_temp.x = op_dist_outcome (uniform_dist_address);
					  address_dest_temp.y = op_dist_outcome (uniform_dist_address);
					  } while (address_dest_temp.x == address_source_temp.x && address_dest_temp.y == address_source_temp.y);
				 break;
			
			 case 2 :
			         {
				//	  printf(" state 1 : local!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
					  address_dest_temp.x = 3 ;
					  address_dest_temp.y = 3 ;
					
					  } 
					 break ;
			
					 
		     default: break;
					 
					 
				}
				
		
		op_dist_unload(uniform_dist_percent);
		op_dist_unload(uniform_dist_address);
		
		return(address_dest_temp );
		
		}

	 
	 
	 
	 
	 
	 
/* 定义热点2流量模型 ，热点为中心区域*/
	 
static T_ADDRESS hotpot_traffic_type_area(T_ADDRESS address_gen, int dim)
	
	
	{
           T_ADDRESS address_source_temp;
	       T_ADDRESS address_dest_temp;
		
	       int dim_temp;
		  
		   int ratio;  
	       Distribution *uniform_dist_percent;
		   Distribution *uniform_dist_address;
		   int limit_up;
		   int limit_down;
		   
		   address_source_temp.x = address_gen.x;		
		   address_source_temp.y = address_gen.y;
		   
		   dim_temp = dim;
 
		   uniform_dist_percent = op_dist_load( "uniform_int", 0, 99);
		   ratio = op_dist_outcome(uniform_dist_percent );
           if (ratio < 10)
				{
                   limit_down = (dim_temp/2)-1;
				   limit_up = dim_temp/2;
							
						
				}/* end if(ratio < 10 ) */
		   else  
			    { 
						
                   limit_down = 0;
			       limit_up = dim_temp-1;
					
				} /* end else  */
					
					
		  
		uniform_dist_address = op_dist_load("uniform_int", limit_down, limit_up);
		do
			{
			
			
			  address_dest_temp.x = op_dist_outcome(uniform_dist_address);
			  
			  address_dest_temp.y = op_dist_outcome(uniform_dist_address);
			  
			  
			  } while(address_dest_temp.x == address_source_temp.x && address_dest_temp.y == address_source_temp.y);
		
		op_dist_unload(uniform_dist_percent);
		op_dist_unload(uniform_dist_address);
      
     	return(address_dest_temp);
		 
	}
	 
	 
	 
	 
	 
	 
/* 定义局部流量模型 */
static T_ADDRESS local_traffic(T_ADDRESS address_gen, int dim)
	{    
           T_ADDRESS address_source_temp;
	       T_ADDRESS address_dest_temp;
		
	       int  dim_temp;
		   Distribution *uniform_dist_percent;
		   Distribution *uniform_dist_address;
           int  ratio = -1;
		   
		   
		   address_source_temp.x = address_gen.x;
		   address_source_temp.y = address_gen.y;
  
		   dim_temp = dim ;
		   
		   uniform_dist_percent = op_dist_load ("uniform_int", 0, 99) ;
		   ratio = op_dist_outcome( uniform_dist_percent );

		   uniform_dist_address = op_dist_load ("uniform_int", 0, dim_temp-1);
		   
		   
		    if (ratio < 30) 
				{
			     do {
				 
				
				    address_dest_temp.x = op_dist_outcome(uniform_dist_address);
				    
					address_dest_temp.y = op_dist_outcome(uniform_dist_address);
		
					 } while((address_dest_temp.x + address_dest_temp.y - address_source_temp.x - address_source_temp.y > 1 )|| (  address_dest_temp.x + address_dest_temp.y - address_source_temp.x - address_source_temp.y< -1)|| (address_dest_temp.x == address_source_temp.x &&address_dest_temp.y == address_source_temp.y));
		                      
                  }
		    else
				 
				 
				 {
				 
				 do {
				    address_dest_temp.x = op_dist_outcome(uniform_dist_address);
				    
					address_dest_temp.y = op_dist_outcome(uniform_dist_address);
					
					
					}while((( address_dest_temp.x + address_dest_temp.y - address_source_temp.x - address_source_temp.y) == 1 ) ||( ( address_dest_temp.x + address_dest_temp.y - address_source_temp.x - address_source_temp.y) == -1)|| ( (address_dest_temp.x == address_source_temp.x) && (address_dest_temp.y == address_source_temp.y)));
		                      
				} 
		   

           op_dist_unload(uniform_dist_percent);
		   op_dist_unload(uniform_dist_address);
		  

		   return(address_dest_temp);
		   
		   }	 
	 
	 
	 
	 
	 
/* 定义比特反转流量模型,适用与IP核个数为2的幂时，如 2,4,8,16..........*/
static T_ADDRESS bit_reverse(T_ADDRESS address_gen, int dim)
	 {
        T_ADDRESS address_source_temp;
	    T_ADDRESS address_dest_temp;
		int dim_temp;
		
		
		address_source_temp.x = address_gen.x;
		address_source_temp.y = address_gen.y;
		dim_temp = dim;
		
		
		
		address_dest_temp.x = (address_source_temp.x) ^ ( dim_temp - 1 );
		address_dest_temp.y = (address_source_temp.y) ^ ( dim_temp - 1 );

		return(address_dest_temp);
		}




	 
	 
	 
	 
/*定义分组产生函数*/
static void ss_packet_generate ()
	{
	Packet *pkptr_optical;
	Packet *pkptr_pathsetup;	
	
	int dest_addr_x_temp,dest_addr_y_temp;
	int dim;
	int i=0,j=0;//用于循环的变量
	int k=0,m=0;
	
	T_ADDRESS address_dest;
	T_ADDRESS address_gen;
	
	
	FIN (ss_packet_generate ());
	
	
	/* 产生信息分组 ，以光标记*/
	
	pkptr_optical = op_pk_create (OP_length);
	
	op_pk_encap_flag_set (pkptr_optical, OPTICAL_FLAG);
	
	op_pk_fd_set (pkptr_optical, ID_NO, OPC_FIELD_TYPE_INTEGER, id_num_global, 15);
	
	op_pk_fd_set (pkptr_optical, SOUR_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, local_node_address[0], 4);
	
	op_pk_fd_set (pkptr_optical, SOUR_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, local_node_address[1], 4);
	
	op_pk_fd_set (pkptr_optical, SOUR_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, local_node_address[2], 4);
	
	
	/* 将源地址写入结构体address_gen*/
	address_gen.x = local_node_address[0];
    address_gen.y = local_node_address[1];	
	dim = Dim;
	
	/*  根据流量模型选择调用相应函数*/
	switch(traffic_module)
   {
    
	case UNIFORM_TRAFFIC:                                                       /*调用均匀分布函数*/
	address_dest = uniform_dist_gen(address_gen,dim);break;	
	
	case MATRIX_REVERSE:                                                       /*调用阵转置分布函数 */
	address_dest = matrix_reverse(address_gen, dim);break;
	
	case HOTPOT_TRAFFIC_POINT:
	
	address_dest = hotpot_traffic_type_point(address_gen, dim); break;         
	
	
	case HOTPOT_TRAFFIC_AREA :
	address_dest = hotpot_traffic_type_area(address_gen, dim);break; 
		
	
	
	case LOCAL_TRAFFIC :
	address_dest = local_traffic(address_gen, dim);break; 
	
	
	case BIT_REVERSE :                                                              
	address_dest = bit_reverse(address_gen , dim); break;
	
	
	case  POINT_TO_POINT_TRAFFIC :                                                    
	       {
		    address_dest.x =0;
			
		    address_dest.y =4;

			}
		   break;
	

	
	default: break;
	}

	
	dest_addr_x_temp = address_dest.x;
	dest_addr_y_temp = address_dest.y;
/*目的节点的个数的统计*/	
/*    for(i=0;i<8;i++)
		{
			for(j=0;j<8;j++)
				{
					if(dest_addr_x_temp==i&&dest_addr_y_temp==j)
						DestRcv[i][j]+=1;
				}	
		}
//记录每个源节点向目的节点发送的个数	
	 for(i=0;i<8;i++)
		{
			for(j=0;j<8;j++)
				{
				for(k=0;k<8;k++)
					{
					for(m=0;m<8;m++)
						{
					if(dest_addr_x_temp==k&&dest_addr_y_temp==m&&local_node_address[0]==i&&local_node_address[1]==j)
						RecToRcv[i][j][k][m]+=1;
						}
					}
				}	
		}
*/
	
	op_pk_fd_set(pkptr_optical, DEST_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, dest_addr_x_temp, 4);
	
	op_pk_fd_set(pkptr_optical, DEST_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, dest_addr_y_temp, 4);
	
	
	/*  将其余域值置为0  */
	
	op_pk_fd_set(pkptr_optical, DEST_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, 0, 4);
	
	op_pk_fd_set(pkptr_optical, ELEC_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, 0.0, 20);
	
	op_pk_fd_set(pkptr_optical, OPT_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, 0.0, 20);
	
	op_pk_fd_set(pkptr_optical, OPT_LOSS_FIELD, OPC_FIELD_TYPE_DOUBLE,  0.0, 10);
	
	op_pk_fd_set(pkptr_optical, HOP_FIELD, OPC_FIELD_TYPE_INTEGER,0, 5);
	

	op_pk_total_size_set(pkptr_optical, OP_length); /*设置整个光包的大小*/
	
	
	
	/* 为上述信息分组产生相应的建链分组，分组的域值相同 */
	
	pkptr_pathsetup = op_pk_create(path_setup_length);
	
	op_pk_encap_flag_set(pkptr_pathsetup, PATHSETUP_FLAG);
	
	op_pk_fd_set(pkptr_pathsetup, ID_NO, OPC_FIELD_TYPE_INTEGER, id_num_global, 15);
	
	op_pk_fd_set(pkptr_pathsetup, SOUR_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, local_node_address[0], 4);
	
	op_pk_fd_set(pkptr_pathsetup, SOUR_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, local_node_address[1], 4);
	
	op_pk_fd_set(pkptr_pathsetup, SOUR_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, local_node_address[2], 4);
	
	
	
	op_pk_fd_set(pkptr_pathsetup, DEST_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, dest_addr_x_temp, 4);
	
	op_pk_fd_set(pkptr_pathsetup, DEST_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, dest_addr_y_temp, 4);
	
	/*  将其余域值置为   0  */
	
	op_pk_fd_set(pkptr_pathsetup, DEST_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, 0, 4);
	
	op_pk_fd_set(pkptr_pathsetup, ELEC_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, 0.0, 20);
	
	op_pk_fd_set(pkptr_pathsetup, HOP_FIELD, OPC_FIELD_TYPE_INTEGER, 0, 5);
	
	op_pk_total_size_set(pkptr_pathsetup, path_setup_length);

	/* 将建链分组先发送出去 */
	op_pk_send(pkptr_pathsetup, 0);
	
	/*然后发送信息分组 */
	op_pk_send(pkptr_optical, 0);
	
/* 记录分组的产生个数      */
	gen_pkts++;
	
	id_num_global++;
	
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
	void Ip_Source_mesh (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_Ip_Source_mesh_init (int * init_block_ptr);
	void _op_Ip_Source_mesh_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_Ip_Source_mesh_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_Ip_Source_mesh_alloc (VosT_Obtype, int);
	void _op_Ip_Source_mesh_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
Ip_Source_mesh (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (Ip_Source_mesh ());

		{


		FSM_ENTER ("Ip_Source_mesh")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (init) enter executives **/
			FSM_STATE_ENTER_UNFORCED_NOLABEL (0, "init", "Ip_Source_mesh [init enter execs]")
				FSM_PROFILE_SECTION_IN ("Ip_Source_mesh [init enter execs]", state0_enter_exec)
				{
				/* 得到模块的对象ID*/
				own_objid = op_id_self ();
				/* 得到该节点的对象ID*/
				node_objid = op_topo_parent(own_objid);
				
				
				
				/* 得到该节点的属性*/
				op_ima_obj_attr_get(node_objid, "Node_Address_X", &local_node_address[0]);
				op_ima_obj_attr_get(node_objid, "Node_Address_Y", &local_node_address[1]);
				op_ima_obj_attr_get(node_objid, "Node_Address_Z", &local_node_address[2]);
				op_ima_obj_attr_get(node_objid, "Send Flag", &send_flag);
				op_ima_obj_attr_get(node_objid, "Collect_Flag", &END_PER);
				
				
				
				/*  得到仿真参数配置*/
				
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Optical Fixed Packet Length(bit)", &OP_length);
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Offered Load", &Offered_load);
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Transmission Bandwidth (Gbps)", &transmission_bandwidth);
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Dim Of Mesh", &Dim);
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Path Setup Length(bit)", &path_setup_length);
				op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Ack Length(bit)",&ack_length);
				
				op_ima_sim_attr_get(OPC_IMA_INTEGER, "Node_Traffic_Module", &traffic_module);                  /* 得到IP 流量模型*/
				
				
				/*设置分组的生成间隔*/
				mean_pk_arrival_time = ((1-Offered_load)*OP_length)/(Offered_load*transmission_bandwidth);
				
				
				/*设置自中断*/
				op_intrpt_schedule_self(op_sim_time (), START);
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (1,"Ip_Source_mesh")


			/** state (init) exit executives **/
			FSM_STATE_EXIT_UNFORCED (0, "init", "Ip_Source_mesh [init exit execs]")


			/** state (init) transition processing **/
			FSM_TRANSIT_ONLY ((SSC_START), 1, state1_enter_exec, ;, init, "SSC_START", "", "init", "pk_generate", "tr_6", "Ip_Source_mesh [init -> pk_generate : SSC_START / ]")
				/*---------------------------------------------------------*/



			/** state (pk_generate) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "pk_generate", state1_enter_exec, "Ip_Source_mesh [pk_generate enter execs]")
				FSM_PROFILE_SECTION_IN ("Ip_Source_mesh [pk_generate enter execs]", state1_enter_exec)
				{
				/*分组的产生间隔服从指数分布，设置中断产生分组*/
				if (send_flag == 1)
					{
					ss_packet_generate();
				
					next_intarr_time = op_dist_exponential(mean_pk_arrival_time);
				
					next_pk_evh = op_intrpt_schedule_self(op_sim_time () + next_intarr_time + OP_length/transmission_bandwidth, GENERATE);
					
				
					}
				
					
				}
				FSM_PROFILE_SECTION_OUT (state1_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"Ip_Source_mesh")


			/** state (pk_generate) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "pk_generate", "Ip_Source_mesh [pk_generate exit execs]")


			/** state (pk_generate) transition processing **/
			FSM_PROFILE_SECTION_IN ("Ip_Source_mesh [pk_generate trans conditions]", state1_trans_conds)
			FSM_INIT_COND (PACKET_GENERATE)
			FSM_TEST_COND (END_SIM)
			FSM_TEST_LOGIC ("pk_generate")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 1, state1_enter_exec, ;, "PACKET_GENERATE", "", "pk_generate", "pk_generate", "tr_7", "Ip_Source_mesh [pk_generate -> pk_generate : PACKET_GENERATE / ]")
				FSM_CASE_TRANSIT (1, 2, state2_enter_exec, ;, "END_SIM", "", "pk_generate", "stati_collect", "tr_9", "Ip_Source_mesh [pk_generate -> stati_collect : END_SIM / ]")
				}
				/*---------------------------------------------------------*/



			/** state (stati_collect) enter executives **/
			FSM_STATE_ENTER_UNFORCED (2, "stati_collect", state2_enter_exec, "Ip_Source_mesh [stati_collect enter execs]")
				FSM_PROFILE_SECTION_IN ("Ip_Source_mesh [stati_collect enter execs]", state2_enter_exec)
				{
				/*统计数据*/
				if (op_ev_valid(next_pk_evh) == OPC_TRUE)
					{
					op_ev_cancel(next_pk_evh);	
					}
				
				if (END_PER == 1)
					{
					int i=0,j=0;
					int k=0,m=0;
					int flag=0;//用于输出换行判断
				/*	//输出目的节点包数目
					for(i=0;i<8;i++)
						{
						for(j=0;j<8;j++)
							{
								printf("目的节点（%d,%d,0)%d ",i,j,DestRcv[i][j]);
							}	
						printf("\n");
						}
				*/
					//输出目的节点接收到每个源节点的包的个数
				/*	 for(i=0;i<8;i++)
						{
							for(j=0;j<8;j++)
								{
								for(k=0;k<8;k++)
									{
									for(m=0;m<8;m++)
										{	
										if(RecToRcv[i][j][k][m]!=0)
											{
											printf("源节点（%d,%d,0）向目的节点（%d,%d,0）发送%d个 \n",i,j,k,m,RecToRcv[i][j][k][m]);
											if(flag==2)
												{
												printf("\n");
												flag=0;
												}
											flag+=flag;
											}
										}
									}
								}	
						}
					printf("总的包数目：%d\n",gen_pkts);
				*/
					printf("产生总的包数目：%d\n",gen_pkts);
					op_stat_scalar_write("PK Generated",   gen_pkts);
					}
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (5,"Ip_Source_mesh")


			/** state (stati_collect) exit executives **/
			FSM_STATE_EXIT_UNFORCED (2, "stati_collect", "Ip_Source_mesh [stati_collect exit execs]")


			/** state (stati_collect) transition processing **/
			FSM_TRANSIT_MISSING ("stati_collect")
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"Ip_Source_mesh")
		}
	}




void
_op_Ip_Source_mesh_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
	/* No Diagnostic Block */
	}




void
_op_Ip_Source_mesh_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_Ip_Source_mesh_terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_Ip_Source_mesh_svar function. */
#undef local_node_address
#undef own_objid
#undef node_objid
#undef next_pk_evh
#undef next_intarr_time
#undef mean_pk_arrival_time
#undef send_flag
#undef END_PER
#undef OP_length
#undef Offered_load
#undef transmission_bandwidth
#undef Dim
#undef traffic_module
#undef ack_length
#undef path_setup_length

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_Ip_Source_mesh_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_Ip_Source_mesh_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (Ip_Source_mesh)",
		sizeof (Ip_Source_mesh_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_Ip_Source_mesh_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	Ip_Source_mesh_state * ptr;
	FIN_MT (_op_Ip_Source_mesh_alloc (obtype))

	ptr = (Ip_Source_mesh_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "Ip_Source_mesh [init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_Ip_Source_mesh_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	Ip_Source_mesh_state		*prs_ptr;

	FIN_MT (_op_Ip_Source_mesh_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (Ip_Source_mesh_state *)gen_ptr;

	if (strcmp ("local_node_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->local_node_address);
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
	if (strcmp ("next_pk_evh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->next_pk_evh);
		FOUT
		}
	if (strcmp ("next_intarr_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->next_intarr_time);
		FOUT
		}
	if (strcmp ("mean_pk_arrival_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->mean_pk_arrival_time);
		FOUT
		}
	if (strcmp ("send_flag" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->send_flag);
		FOUT
		}
	if (strcmp ("END_PER" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->END_PER);
		FOUT
		}
	if (strcmp ("OP_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->OP_length);
		FOUT
		}
	if (strcmp ("Offered_load" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->Offered_load);
		FOUT
		}
	if (strcmp ("transmission_bandwidth" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->transmission_bandwidth);
		FOUT
		}
	if (strcmp ("Dim" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->Dim);
		FOUT
		}
	if (strcmp ("traffic_module" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->traffic_module);
		FOUT
		}
	if (strcmp ("ack_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ack_length);
		FOUT
		}
	if (strcmp ("path_setup_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->path_setup_length);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

