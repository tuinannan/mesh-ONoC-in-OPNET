MIL_3_Tfile_Hdr_ 145A 140A modeler 9 4C5FC5A3 56E6C91C 3D CN13549845642 Administrator 0 0 none none 0 0 none 265B6CF 5240 0 0 0 0 0 0 1bcc 1                                                                                                                                                                                                                                                                                                                                                                                    ��g�      @   D  I      A�  P  P   P$  P(  P4  P8  P<  A�           Optical Fixed Packet Length(bit)   �������   ����               ����              ����              ����           �Z             Offered Load   �������   ����               ����              ����              ����           �Z             Transmission Bandwidth (Gbps)   �������   ����               ����              ����              ����           �Z             Dim Of Mesh   �������   ����               ����              ����              ����           �Z             Number Of Out Port   �������   ����               ����              ����              ����           �Z             Path Setup Length(bit)   �������   ����               ����              ����              ����           �Z             Roll Around Period(ns)   �������   ����               ����              ����              ����           �Z             Ack Length(bit)   �������   ����               ����              ����              ����           �Z             	   begsim intrpt             ����      doc file            	nd_module      endsim intrpt             ����      failure intrpts            disabled      intrpt interval         ԲI�%��}����      priority              ����      recovery intrpts            disabled      subqueue                     count    ���   
   ����   
      list   	���   
          
      super priority         
    ����   
          int	\local_node_address[3];       Objid	\own_objid;       Objid	\node_objid;       Evhandle	\next_pk_evh;       double	\next_intarr_time;       double	\mean_pk_arrival_time;       int	\send_flag;       int	\END_PER;       double	\OP_length;       double	\Offered_load;       double	\transmission_bandwidth;       double	\Dim;       int	\traffic_module;       double	\ack_length;       double	\path_setup_length;           a       #include "opnet.h"   #include "math.h"   #include <stdio.h>           /*�����ж���*/   #define	 START			0   #define	 GENERATE			1               /*����״̬ת������*/   =#define     SSC_START             (op_intrpt_code() == START)   I#define     END_SIM               (op_intrpt_type() == OPC_INTRPT_ENDSIM)   e#define		PACKET_GENERATE	      (op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code()== GENERATE)               /*���������*/   #define ID_NO				  0	             #define DEST_ADDR_FIELD_X     1   #define DEST_ADDR_FIELD_Y     2   #define DEST_ADDR_FIELD_Z     3   #define SOUR_ADDR_FIELD_X	  4   #define SOUR_ADDR_FIELD_Y	  5   #define SOUR_ADDR_FIELD_Z	  6           /*  �����ӵİ�����  */   ;#define ELEC_POWER_FIELD              7                       ;#define OPT_POWER_FIELD               8                       :#define OPT_LOSS_FIELD                9                      =#define HOP_FIELD                     10                                /* ����ʱ��Ϳռ������ģ�� */       %#define   UNIFORM_TRAFFIC           1   %#define   MATRIX_REVERSE            2   %#define   HOTPOT_TRAFFIC_POINT      3   %#define   HOTPOT_TRAFFIC_AREA       4   %#define   LOCAL_TRAFFIC             5   %#define   BIT_REVERSE               6   %#define   POINT_TO_POINT_TRAFFIC    7                                           /*���岻ͬ����ı�ʶ*/   #define PATHSETUP_FLAG        0   #define OPTICAL_FLAG          1   #define ACK_FLAG              2               /*����ͳ�Ʊ���*/   int id_num_global = 0;   int gen_pkts=0;   @int DestRcv[8][8]={{0},{0},{0}};//ͳ��ÿ��Ŀ�Ľڵ���յ����ĸ���   Iint RecToRcv[8][8][8][8]={0};  //ͳ��Ŀ�Ľڵ��ÿ��Դ�ڵ���յ��İ��ĸ���                   /*�������Ŀ�ĵ�ַ�Ľṹ��*/   typedef struct    		{ int x;   		  int y;   	  } T_ADDRESS;               /*��������*/           Bstatic T_ADDRESS uniform_dist_gen(T_ADDRESS address_gen, int dim);       @static T_ADDRESS matrix_reverse(T_ADDRESS address_gen, int dim);       Kstatic T_ADDRESS hotpot_traffic_type_point(T_ADDRESS address_gen, int dim);       Jstatic T_ADDRESS hotpot_traffic_type_area(T_ADDRESS address_gen, int dim);       ?static T_ADDRESS local_traffic(T_ADDRESS address_gen, int dim);       =static T_ADDRESS bit_reverse(T_ADDRESS address_gen, int dim);               !static void ss_packet_generate();  �       /* �����������ģ�� */   B static T_ADDRESS uniform_dist_gen(T_ADDRESS address_gen, int dim)   	      	    	{    #	    T_ADDRESS address_source_temp;   !	    T_ADDRESS address_dest_temp;   		   	    int dim_temp;   		Distribution* uniform_dist;   		dim_temp = dim;   *		address_source_temp.x = address_gen.x;		   (		address_source_temp.y = address_gen.y;   H		uniform_dist = op_dist_load("uniform_int",0,dim_temp-1);//���طֲ�����   		do   			{   9			  address_dest_temp.x = op_dist_outcome(uniform_dist);   9			  address_dest_temp.y = op_dist_outcome(uniform_dist);   			     i			} while(address_dest_temp.x == address_source_temp.x && address_dest_temp.y == address_source_temp.y);   		   )		op_dist_unload(uniform_dist);//�ͷ��ڴ�   	   		return(address_dest_temp);   		   	 }   	    	    	    	    	 	    /* �������ת������ģ�� */       ?static T_ADDRESS matrix_reverse(T_ADDRESS address_gen, int dim)   	         	{    #	    T_ADDRESS address_source_temp;   !	    T_ADDRESS address_dest_temp;   		   		int dim_temp;   		dim_temp = dim;   *		address_source_temp.x = address_gen.x;		   (		address_source_temp.y = address_gen.y;   9		address_dest_temp.x = dim_temp-address_source_temp.x-1;   9		address_dest_temp.y = dim_temp-address_source_temp.y-1;       		return(address_dest_temp);	   	}   				   	   -/* �����ȵ�����ģ��,�涨ĳ���̶��ڵ�Ϊ�ȵ� */   	    L static T_ADDRESS hotpot_traffic_type_point(T_ADDRESS address_gen, int dim )   	 {    #	    T_ADDRESS address_source_temp;   !	    T_ADDRESS address_dest_temp;   		int dim_temp;   		int state = -1;   		int ratio;   		   %		Distribution* uniform_dist_percent;   %		Distribution* uniform_dist_address;   		   		dim_temp = dim ;   )		address_source_temp.x = address_gen.x ;   )		address_source_temp.y = address_gen.y ;   		   <		uniform_dist_percent = op_dist_load( "uniform_int" ,0,99);   3		ratio = op_dist_outcome( uniform_dist_percent ) ;   		   		if( ratio < 10 )  state = 2 ;   		else             state = 1 ;   		   		   		   G		uniform_dist_address = op_dist_load ( "uniform_int" , 0, dim_temp-1);   		switch (state)   			{   			 case 1 :   		         do   					 {    					    K			//		  printf(" state 1 : global!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");   D					  address_dest_temp.x = op_dist_outcome (uniform_dist_address);   D					  address_dest_temp.y = op_dist_outcome (uniform_dist_address);   n					  } while (address_dest_temp.x == address_source_temp.x && address_dest_temp.y == address_source_temp.y);   				 break;   			   			 case 2 :   			         {   J				//	  printf(" state 1 : local!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");    					  address_dest_temp.x = 3 ;    					  address_dest_temp.y = 3 ;   					   						  }    					 break ;   			   					    		     default: break;   					    					    				}   				   		   '		op_dist_unload(uniform_dist_percent);   '		op_dist_unload(uniform_dist_address);   		   		return(address_dest_temp );   		   		}       	    	    	    	    	    	    '/* �����ȵ�2����ģ�� ���ȵ�Ϊ��������*/   	    Istatic T_ADDRESS hotpot_traffic_type_area(T_ADDRESS address_gen, int dim)   	   	   	{   )           T_ADDRESS address_source_temp;   $	       T_ADDRESS address_dest_temp;   		   	       int dim_temp;   		     		   int ratio;     +	       Distribution *uniform_dist_percent;   (		   Distribution *uniform_dist_address;   		   int limit_up;   		   int limit_down;   		      -		   address_source_temp.x = address_gen.x;		   +		   address_source_temp.y = address_gen.y;   		      		   dim_temp = dim;       @		   uniform_dist_percent = op_dist_load( "uniform_int", 0, 99);   4		   ratio = op_dist_outcome(uniform_dist_percent );              if (ratio < 10)   				{   /                   limit_down = (dim_temp/2)-1;   				   limit_up = dim_temp/2;   							   						   				}/* end if(ratio < 10 ) */   		   else     				    {    						   "                   limit_down = 0;    			       limit_up = dim_temp-1;   					   				} /* end else  */   					   					   		     K		uniform_dist_address = op_dist_load("uniform_int", limit_down, limit_up);   		do   			{   			   			   A			  address_dest_temp.x = op_dist_outcome(uniform_dist_address);   			     A			  address_dest_temp.y = op_dist_outcome(uniform_dist_address);   			     			     k			  } while(address_dest_temp.x == address_source_temp.x && address_dest_temp.y == address_source_temp.y);   		   '		op_dist_unload(uniform_dist_percent);   '		op_dist_unload(uniform_dist_address);                  	return(address_dest_temp);   		    	}   	    	    	    	    	    	    /* ����ֲ�����ģ�� */   >static T_ADDRESS local_traffic(T_ADDRESS address_gen, int dim)   	{       )           T_ADDRESS address_source_temp;   $	       T_ADDRESS address_dest_temp;   		   	       int  dim_temp;   (		   Distribution *uniform_dist_percent;   (		   Distribution *uniform_dist_address;              int  ratio = -1;   		      		      +		   address_source_temp.x = address_gen.x;   +		   address_source_temp.y = address_gen.y;        		   dim_temp = dim ;   		      A		   uniform_dist_percent = op_dist_load ("uniform_int", 0, 99) ;   5		   ratio = op_dist_outcome( uniform_dist_percent );       H		   uniform_dist_address = op_dist_load ("uniform_int", 0, dim_temp-1);   		      		      		    if (ratio < 30)    				{   			     do {   				    				   D				    address_dest_temp.x = op_dist_outcome(uniform_dist_address);   				       A					address_dest_temp.y = op_dist_outcome(uniform_dist_address);   		  4					 } while((address_dest_temp.x + address_dest_temp.y - address_source_temp.x - address_source_temp.y > 1 )|| (  address_dest_temp.x + address_dest_temp.y - address_source_temp.x - address_source_temp.y< -1)|| (address_dest_temp.x == address_source_temp.x &&address_dest_temp.y == address_source_temp.y));   		                                           }   
		    else   				    				    				 {   				    					 do {   D				    address_dest_temp.x = op_dist_outcome(uniform_dist_address);   				       A					address_dest_temp.y = op_dist_outcome(uniform_dist_address);   					   					  @					}while((( address_dest_temp.x + address_dest_temp.y - address_source_temp.x - address_source_temp.y) == 1 ) ||( ( address_dest_temp.x + address_dest_temp.y - address_source_temp.x - address_source_temp.y) == -1)|| ( (address_dest_temp.x == address_source_temp.x) && (address_dest_temp.y == address_source_temp.y)));   		                         				}    		          0           op_dist_unload(uniform_dist_percent);   *		   op_dist_unload(uniform_dist_address);   		         		   return(address_dest_temp);   		      		   }	    	    	    	    	    	    H/* ������ط�ת����ģ��,������IP�˸���Ϊ2����ʱ���� 2,4,8,16..........*/   <static T_ADDRESS bit_reverse(T_ADDRESS address_gen, int dim)   	 {   &        T_ADDRESS address_source_temp;   !	    T_ADDRESS address_dest_temp;   		int dim_temp;   		   		   (		address_source_temp.x = address_gen.x;   (		address_source_temp.y = address_gen.y;   		dim_temp = dim;   		   		   		   C		address_dest_temp.x = (address_source_temp.x) ^ ( dim_temp - 1 );   C		address_dest_temp.y = (address_source_temp.y) ^ ( dim_temp - 1 );       		return(address_dest_temp);   		}                   	    	    	    	    /*��������������*/   !static void ss_packet_generate ()   	{   	Packet *pkptr_optical;   	Packet *pkptr_pathsetup;	   	   '	int dest_addr_x_temp,dest_addr_y_temp;   		int dim;   	int i=0,j=0;//����ѭ���ı���   	int k=0,m=0;   	   	T_ADDRESS address_dest;   	T_ADDRESS address_gen;   	   	   	FIN (ss_packet_generate ());   	   	   	/* ������Ϣ���� ���Թ���*/   	   *	pkptr_optical = op_pk_create (OP_length);   	   4	op_pk_encap_flag_set (pkptr_optical, OPTICAL_FLAG);   	   P	op_pk_fd_set (pkptr_optical, ID_NO, OPC_FIELD_TYPE_INTEGER, id_num_global, 15);   	   c	op_pk_fd_set (pkptr_optical, SOUR_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, local_node_address[0], 4);   	   c	op_pk_fd_set (pkptr_optical, SOUR_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, local_node_address[1], 4);   	   c	op_pk_fd_set (pkptr_optical, SOUR_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, local_node_address[2], 4);   	   	   #	/* ��Դ��ַд��ṹ��address_gen*/   '	address_gen.x = local_node_address[0];   +    address_gen.y = local_node_address[1];	   	dim = Dim;   	   #	/*  ��������ģ��ѡ�������Ӧ����*/   	switch(traffic_module)      {          a	case UNIFORM_TRAFFIC:                                                       /*���þ��ȷֲ�����*/   9	address_dest = uniform_dist_gen(address_gen,dim);break;	   	   c	case MATRIX_REVERSE:                                                       /*������ת�÷ֲ����� */   7	address_dest = matrix_reverse(address_gen, dim);break;   	   	case HOTPOT_TRAFFIC_POINT:   	   L	address_dest = hotpot_traffic_type_point(address_gen, dim); break;            	   	   	case HOTPOT_TRAFFIC_AREA :   B	address_dest = hotpot_traffic_type_area(address_gen, dim);break;    		   	   	   	case LOCAL_TRAFFIC :   7	address_dest = local_traffic(address_gen, dim);break;    	   	   Q	case BIT_REVERSE :                                                                 6	address_dest = bit_reverse(address_gen , dim); break;   	   	   S	case  POINT_TO_POINT_TRAFFIC :                                                       		       {   		    address_dest.x =0;   			   		    address_dest.y =4;       			}   		   break;   	       	   	default: break;   	}       	   #	dest_addr_x_temp = address_dest.x;   #	dest_addr_y_temp = address_dest.y;   /*Ŀ�Ľڵ�ĸ�����ͳ��*/	   /*    for(i=0;i<8;i++)   		{   			for(j=0;j<8;j++)   				{   1					if(dest_addr_x_temp==i&&dest_addr_y_temp==j)   						DestRcv[i][j]+=1;   				}	   		}   %//��¼ÿ��Դ�ڵ���Ŀ�Ľڵ㷢�͵ĸ���	   	 for(i=0;i<8;i++)   		{   			for(j=0;j<8;j++)   				{   				for(k=0;k<8;k++)   					{   					for(m=0;m<8;m++)   						{   e					if(dest_addr_x_temp==k&&dest_addr_y_temp==m&&local_node_address[0]==i&&local_node_address[1]==j)   						RecToRcv[i][j][k][m]+=1;   						}   					}   				}	   		}   */   	   ]	op_pk_fd_set(pkptr_optical, DEST_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, dest_addr_x_temp, 4);   	   ]	op_pk_fd_set(pkptr_optical, DEST_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, dest_addr_y_temp, 4);   	   	   	/*  ��������ֵ��Ϊ0  */   	   N	op_pk_fd_set(pkptr_optical, DEST_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, 0, 4);   	   O	op_pk_fd_set(pkptr_optical, ELEC_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, 0.0, 20);   	   N	op_pk_fd_set(pkptr_optical, OPT_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, 0.0, 20);   	   N	op_pk_fd_set(pkptr_optical, OPT_LOSS_FIELD, OPC_FIELD_TYPE_DOUBLE,  0.0, 10);   	   E	op_pk_fd_set(pkptr_optical, HOP_FIELD, OPC_FIELD_TYPE_INTEGER,0, 5);   	       G	op_pk_total_size_set(pkptr_optical, OP_length); /*������������Ĵ�С*/   	   	   	   7	/* Ϊ������Ϣ���������Ӧ�Ľ������飬�������ֵ��ͬ */   	   3	pkptr_pathsetup = op_pk_create(path_setup_length);   	   7	op_pk_encap_flag_set(pkptr_pathsetup, PATHSETUP_FLAG);   	   Q	op_pk_fd_set(pkptr_pathsetup, ID_NO, OPC_FIELD_TYPE_INTEGER, id_num_global, 15);   	   d	op_pk_fd_set(pkptr_pathsetup, SOUR_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, local_node_address[0], 4);   	   d	op_pk_fd_set(pkptr_pathsetup, SOUR_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, local_node_address[1], 4);   	   d	op_pk_fd_set(pkptr_pathsetup, SOUR_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, local_node_address[2], 4);   	   	   	   _	op_pk_fd_set(pkptr_pathsetup, DEST_ADDR_FIELD_X, OPC_FIELD_TYPE_INTEGER, dest_addr_x_temp, 4);   	   _	op_pk_fd_set(pkptr_pathsetup, DEST_ADDR_FIELD_Y, OPC_FIELD_TYPE_INTEGER, dest_addr_y_temp, 4);   	   	/*  ��������ֵ��Ϊ   0  */   	   P	op_pk_fd_set(pkptr_pathsetup, DEST_ADDR_FIELD_Z, OPC_FIELD_TYPE_INTEGER, 0, 4);   	   Q	op_pk_fd_set(pkptr_pathsetup, ELEC_POWER_FIELD, OPC_FIELD_TYPE_DOUBLE, 0.0, 20);   	   H	op_pk_fd_set(pkptr_pathsetup, HOP_FIELD, OPC_FIELD_TYPE_INTEGER, 0, 5);   	   :	op_pk_total_size_set(pkptr_pathsetup, path_setup_length);       	/* �����������ȷ��ͳ�ȥ */    	op_pk_send(pkptr_pathsetup, 0);   	   	/*Ȼ������Ϣ���� */   	op_pk_send(pkptr_optical, 0);   	   /* ��¼����Ĳ�������      */   	gen_pkts++;   	   	id_num_global++;   	   	FOUT;   	   	}               		        		   		                                                  �   �          
   init   
       
   "   /* �õ�ģ��Ķ���ID*/   own_objid = op_id_self ();   /* �õ��ýڵ�Ķ���ID*/   'node_objid = op_topo_parent(own_objid);               /* �õ��ýڵ������*/   Jop_ima_obj_attr_get(node_objid, "Node_Address_X", &local_node_address[0]);   Jop_ima_obj_attr_get(node_objid, "Node_Address_Y", &local_node_address[1]);   Jop_ima_obj_attr_get(node_objid, "Node_Address_Z", &local_node_address[2]);   9op_ima_obj_attr_get(node_objid, "Send Flag", &send_flag);   :op_ima_obj_attr_get(node_objid, "Collect_Flag", &END_PER);               /*  �õ������������*/       Top_ima_sim_attr_get(OPC_IMA_DOUBLE, "Optical Fixed Packet Length(bit)", &OP_length);   Cop_ima_sim_attr_get(OPC_IMA_DOUBLE, "Offered Load", &Offered_load);   ^op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Transmission Bandwidth (Gbps)", &transmission_bandwidth);   9op_ima_sim_attr_get(OPC_IMA_DOUBLE, "Dim Of Mesh", &Dim);   Rop_ima_sim_attr_get(OPC_IMA_DOUBLE, "Path Setup Length(bit)", &path_setup_length);   Cop_ima_sim_attr_get(OPC_IMA_DOUBLE, "Ack Length(bit)",&ack_length);       sop_ima_sim_attr_get(OPC_IMA_INTEGER, "Node_Traffic_Module", &traffic_module);                  /* �õ�IP ����ģ��*/           /*���÷�������ɼ��*/   Zmean_pk_arrival_time = ((1-Offered_load)*OP_length)/(Offered_load*transmission_bandwidth);           /*�������ж�*/   /op_intrpt_schedule_self(op_sim_time (), START);   
                     
    ����   
          pr_state        J   �          
   pk_generate   
       
      0/*����Ĳ����������ָ���ֲ��������жϲ�������*/   if (send_flag == 1)   	{   	ss_packet_generate();       >	next_intarr_time = op_dist_exponential(mean_pk_arrival_time);       w	next_pk_evh = op_intrpt_schedule_self(op_sim_time () + next_intarr_time + OP_length/transmission_bandwidth, GENERATE);   	       	}       	   
                     
    ����   
          pr_state        J  �          
   stati_collect   
       
   1   /*ͳ������*/   )if (op_ev_valid(next_pk_evh) == OPC_TRUE)   	{   	op_ev_cancel(next_pk_evh);	   	}       if (END_PER == 1)   	{   	int i=0,j=0;   	int k=0,m=0;   	int flag=0;//������������ж�   /*	//���Ŀ�Ľڵ����Ŀ   	for(i=0;i<8;i++)   		{   		for(j=0;j<8;j++)   			{   6				printf("Ŀ�Ľڵ㣨%d,%d,0)%d ",i,j,DestRcv[i][j]);   			}	   		printf("\n");   		}   */   )	//���Ŀ�Ľڵ���յ�ÿ��Դ�ڵ�İ��ĸ���   /*	 for(i=0;i<8;i++)   		{   			for(j=0;j<8;j++)   				{   				for(k=0;k<8;k++)   					{   					for(m=0;m<8;m++)   						{	   !						if(RecToRcv[i][j][k][m]!=0)   							{   `							printf("Դ�ڵ㣨%d,%d,0����Ŀ�Ľڵ㣨%d,%d,0������%d�� \n",i,j,k,m,RecToRcv[i][j][k][m]);   							if(flag==2)   									{   								printf("\n");   								flag=0;   									}   							flag+=flag;   							}   						}   					}   				}	   		}   %	printf("�ܵİ���Ŀ��%d\n",gen_pkts);   */   )	printf("�����ܵİ���Ŀ��%d\n",gen_pkts);   2	op_stat_scalar_write("PK Generated",   gen_pkts);   	}   
                         ����             pr_state                       �   �      �   �  -   �          
   tr_6   
       
   	SSC_START   
       ����          
    ����   
          ����                       pr_transition              �   �     a   �  �   �  �   �  c   �          
   tr_7   
       
   PACKET_GENERATE   
       ����          
    ����   
          ����                       pr_transition      	        J  )     J   �  J  k          
   tr_9   
       
   END_SIM   
       ����          
    ����   
          ����                       pr_transition         
                                    