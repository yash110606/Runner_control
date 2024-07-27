#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <std_msgs/msg/int32.h>
#include "driver/gpio.h"
#include <stdio.h>
static rclc_executor_t executor;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_subscription_t subscriber;
static rcl_node_t node;

std_msgs__msg__Int32 msg;
#define M1 33
#define M2 12
#define M3 14
#define FW 27
#define RV 26
#define MS 25
static int old_bin=0;
int bin=0;
int convert_to_binary(int y)
{
	if(y/2 == 0)
	{
		return 1;
	}
	else
	{
		int rem = y%2;
		int z = convert_to_binary(y/2)*10+rem;
		return z;		
	}
}
void switch_on_motor(gpio_num_t mnum, int b)
{
	gpio_set_level(mnum,0);
	if((b%1000)/100==1)
	{
		gpio_set_level(FW,0);
	}
	else if((bin%100)/10==1)
	{
		gpio_set_level(RV,0);
	}
}
void switch_off_motor(gpio_num_t mnum)
{
	gpio_set_level(FW,1);
	gpio_set_level(RV,1);
	vTaskDelay(3000/portTICK_PERIOD_MS);
	gpio_set_level(mnum,1);
}
int controls(int z)
{
	if(old_bin==0)
	{
		if(z%10==1)
		{
			gpio_set_level(MS,0);
			switch_on_motor(M3,z);
		}
		else
		{
			if((z/10000)%10==1)
			{
				switch_on_motor(M2,z);
			}
			else if((z/100000)%10==1)
			{
				switch_on_motor(M1,z);
			}
		}
	}
	else
	{
		if(z%10 != old_bin%10)
		{
			if(z%10==1)
			{
				if(old_bin/10000 != 0)
				{
					gpio_set_level(FW,1);
					gpio_set_level(RV,1);
					vTaskDelay(3000/portTICK_PERIOD_MS);
					gpio_set_level(M1,1);
					gpio_set_level(M2,1);
				}
				gpio_set_level(MS,0);
				switch_on_motor(M3,z);
			}
			else
			{
				if(old_bin/1000 != 0)
				{
					switch_off_motor(M3);
				}
				gpio_set_level(MS,1);
				if((z/10000)%10==1)
				{
					switch_on_motor(M2,z);
				}
				else if((z/100000)==1)
				{
					switch_on_motor(M1,z);
				}
				old_bin = z;
				return 0;
			}
		}
		if(z/1000 !=0)
		{
			if(z/1000 != old_bin/1000)
			{
				if(z%10==1)
				{
					if((z/1000)%10==1)
					{
						gpio_set_level(M3,0);
					}
					else
					{
						switch_off_motor(M3);
					}
				}
				else
				{
					if((z/10000)%10==1)
					{
						if(gpio_get_level(M1)==0)
						{
							switch_off_motor(M1);
						}
						switch_on_motor(M2,z);
						old_bin = z;
						return 0;
					}
					else if((z/100000)==1)
					{
						if(gpio_get_level(M2)==0)
						{
							switch_off_motor(M2);
						}
						switch_on_motor(M1,z);
						old_bin = z;
						return 0;
					}
					else
					{
						gpio_set_level(FW,1);
						gpio_set_level(RV,1);
						vTaskDelay(3000/portTICK_PERIOD_MS);
						gpio_set_level(M1,1);
						gpio_set_level(M2,1);	
					}
				}
			}
			if((z%1000)/10!=0)
			{
				if(z%1000 != old_bin%1000)
				{
					if((z%100)/10==1)
					{
						gpio_set_level(FW,1);
						gpio_set_level(RV,0);
					}
					else
					{
						gpio_set_level(RV,1);
						gpio_set_level(FW,0);
					}
				}

			}
			else
			{
				gpio_set_level(RV,1);
				gpio_set_level(FW,1);
			}
		}
		else
		{
			if(z%10==1)
			{
				switch_off_motor(M3);
			}
			else
			{
				gpio_set_level(FW,1);
				gpio_set_level(RV,1);
				vTaskDelay(3000/portTICK_PERIOD_MS);
				gpio_set_level(M1,1);
				gpio_set_level(M2,1);
			}
		}
	}
	old_bin = z;
	return 0;
}
void init_gpio()
{
	gpio_reset_pin(M1);
	gpio_set_direction(M1,GPIO_MODE_OUTPUT);
	gpio_set_level(M1,1);
	gpio_reset_pin(M2);
	gpio_set_direction(M2,GPIO_MODE_OUTPUT);
	gpio_set_level(M2,1);
	gpio_reset_pin(M3);
	gpio_set_direction(M3,GPIO_MODE_OUTPUT);
	gpio_set_level(M3,1);
	gpio_reset_pin(FW);
	gpio_set_direction(FW,GPIO_MODE_OUTPUT);
	gpio_set_level(FW,1);
	gpio_reset_pin(RV);
	gpio_set_direction(RV,GPIO_MODE_OUTPUT);
	gpio_set_level(RV,1);
	gpio_reset_pin(MS);
	gpio_set_direction(MS,GPIO_MODE_OUTPUT);
	gpio_set_level(MS,1);
}
void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Received: %d\n", msg->data);
	int x = msg->data;
	if(x!=-1)
	{
		bin = convert_to_binary(x);
	}
	if(bin!=old_bin)
	{
		controls(bin);
	}
}

void appMain()
{
	init_gpio();
	allocator = rcl_get_default_allocator();
	// create init_options
	rclc_support_init(&support, 0, NULL, &allocator);

	// create node
	rclc_node_init_default(&node, "int32_subscriber_rclc", "", &support);

	// create subscriber
	rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/microROS/int32_subscriber");

	// create executor
	rclc_executor_init(&executor, &support.context, 1, &allocator);
	rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
	while(1)
	{
		rclc_executor_spin_some(&executor,1000*(1000*100));
	}
	rclc_executor_fini(&executor);
	rcl_node_fini(&node);
}