
#define _GNU_SOURCE
#include <errno.h>
#include "mycan.h"
#include "interface/log/log.h"
#include "interface/G_GloabalVariable.h"
#include <fcntl.h>
#define GET_STATE 1
#define GET_RESTART_MS 2
#define GET_BITTIMING 3
#define GET_CTRLMODE 4
#define GET_CLOCK 5
#define GET_BITTIMING_CONST 6
#define GET_BERR_COUNTER 7
#define GET_XSTATS 8

struct get_req
{
	struct nlmsghdr n;
	struct rtgenmsg g;
};

#include <errno.h>
enum CANFrameType
{
	CAN_FRAME_NONE = 0,
	CAN_FRAME_STANDARD = 1,
	CAN_FRAME_FD = 2
};
#define parse_rtattr_nested(tb, max, rta) \
	(HAL_parse_rtattr((tb), (max), RTA_DATA(rta), RTA_PAYLOAD(rta)))


static int HAL_send_dump_request(int fd, int family, int type)
{
	struct get_req req;

	memset(&req, 0, sizeof(req));

	req.n.nlmsg_len = sizeof(req);
	req.n.nlmsg_type = type;
	req.n.nlmsg_flags = NLM_F_REQUEST | NLM_F_ROOT | NLM_F_MATCH;
	req.n.nlmsg_pid = 0;
	req.n.nlmsg_seq = 0;

	req.g.rtgen_family = family;

	return send(fd, (void *)&req, sizeof(req), 0);
}

static void HAL_parse_rtattr(struct rtattr **tb, int max, struct rtattr *rta, int len)
{
	memset(tb, 0, sizeof(*tb) * max);
	while (RTA_OK(rta, len))
	{
		if (rta->rta_type <= max)
		{
			tb[rta->rta_type] = rta;
		}

		rta = RTA_NEXT(rta, len);
	}
}

static int HAL_do_get_nl_link(int fd, __u8 acquire, const char *name, void *res)
{
	struct sockaddr_nl peer;

	char cbuf[64];
	char nlbuf[1024 * 8];

	int ret = -1;
	int done = 0;

	struct iovec iov = {
		.iov_base = (void *)nlbuf,
		.iov_len = sizeof(nlbuf),
	};

	struct msghdr msg = {
		.msg_name = (void *)&peer,
		.msg_namelen = sizeof(peer),
		.msg_iov = &iov,
		.msg_iovlen = 1,
		.msg_control = &cbuf,
		.msg_controllen = sizeof(cbuf),
		.msg_flags = 0,
	};
	struct nlmsghdr *nl_msg;
	ssize_t msglen;

	struct rtattr *linkinfo[IFLA_INFO_MAX + 1];
	struct rtattr *can_attr[IFLA_CAN_MAX + 1];

	if (HAL_send_dump_request(fd, AF_PACKET, RTM_GETLINK) < 0)
	{
		perror("Cannot send dump request");
		return ret;
	}

	while (!done && (msglen = recvmsg(fd, &msg, 0)) > 0)
	{
		size_t u_msglen = (size_t)msglen;
		/* Check to see if the buffers in msg get truncated */
		if (msg.msg_namelen != sizeof(peer) ||
			(msg.msg_flags & (MSG_TRUNC | MSG_CTRUNC)))
		{
			fprintf(stderr, "Uhoh... truncated message.\n");
			return -1;
		}

		for (nl_msg = (struct nlmsghdr *)nlbuf;
			 NLMSG_OK(nl_msg, u_msglen);
			 nl_msg = NLMSG_NEXT(nl_msg, u_msglen))
		{
			int type = nl_msg->nlmsg_type;
			int len;

			if (type == NLMSG_DONE)
			{
				done++;
				continue;
			}
			if (type != RTM_NEWLINK)
				continue;

			struct ifinfomsg *ifi = NLMSG_DATA(nl_msg);
			struct rtattr *tb[IFLA_MAX + 1];

			len =
				nl_msg->nlmsg_len - NLMSG_LENGTH(sizeof(struct ifaddrmsg));
			HAL_parse_rtattr(tb, IFLA_MAX, IFLA_RTA(ifi), len);

			if (strcmp((char *)RTA_DATA(tb[IFLA_IFNAME]), name) != 0)
				continue;

			if (tb[IFLA_LINKINFO])
				parse_rtattr_nested(linkinfo,
									IFLA_INFO_MAX, tb[IFLA_LINKINFO]);
			else
				continue;

			if (acquire == GET_XSTATS)
			{
				if (!linkinfo[IFLA_INFO_XSTATS])
					fprintf(stderr, "no can statistics found\n");
				else
				{
					memcpy(res, RTA_DATA(linkinfo[IFLA_INFO_XSTATS]),
						   sizeof(struct can_device_stats));
					ret = 0;
				}
				continue;
			}

			if (!linkinfo[IFLA_INFO_DATA])
			{
				fprintf(stderr, "no link data found\n");
				return ret;
			}

			parse_rtattr_nested(can_attr, IFLA_CAN_MAX,
								linkinfo[IFLA_INFO_DATA]);

			switch (acquire)
			{
			case GET_STATE:
				if (can_attr[IFLA_CAN_STATE])
				{
					*((int *)res) = *((__u32 *)
										  RTA_DATA(can_attr
													   [IFLA_CAN_STATE]));
					ret = 0;
				}
				else
				{
					fprintf(stderr, "no state data found\n");
				}

				break;
			case GET_RESTART_MS:
				if (can_attr[IFLA_CAN_RESTART_MS])
				{
					*((__u32 *)res) = *((__u32 *)
											RTA_DATA(can_attr
														 [IFLA_CAN_RESTART_MS]));
					ret = 0;
				}
				else
					fprintf(stderr, "no restart_ms data found\n");

				break;
			case GET_BITTIMING:
				if (can_attr[IFLA_CAN_BITTIMING])
				{
					memcpy(res,
						   RTA_DATA(can_attr[IFLA_CAN_BITTIMING]),
						   sizeof(struct can_bittiming));
					ret = 0;
				}
				else
					fprintf(stderr, "no bittiming data found\n");

				break;
			case GET_CTRLMODE:
				if (can_attr[IFLA_CAN_CTRLMODE])
				{
					memcpy(res,
						   RTA_DATA(can_attr[IFLA_CAN_CTRLMODE]),
						   sizeof(struct can_ctrlmode));
					ret = 0;
				}
				else
					fprintf(stderr, "no ctrlmode data found\n");

				break;
			case GET_CLOCK:
				if (can_attr[IFLA_CAN_CLOCK])
				{
					memcpy(res,
						   RTA_DATA(can_attr[IFLA_CAN_CLOCK]),
						   sizeof(struct can_clock));
					ret = 0;
				}
				else
					fprintf(stderr,
							"no clock parameter data found\n");

				break;
			case GET_BITTIMING_CONST:
				if (can_attr[IFLA_CAN_BITTIMING_CONST])
				{
					memcpy(res,
						   RTA_DATA(can_attr[IFLA_CAN_BITTIMING_CONST]),
						   sizeof(struct can_bittiming_const));
					ret = 0;
				}
				else
					fprintf(stderr, "no bittiming_const data found\n");

				break;
			case GET_BERR_COUNTER:
				if (can_attr[IFLA_CAN_BERR_COUNTER])
				{
					memcpy(res,
						   RTA_DATA(can_attr[IFLA_CAN_BERR_COUNTER]),
						   sizeof(struct can_berr_counter));
					ret = 0;
				}
				else
					fprintf(stderr, "no berr_counter data found\n");

				break;

			default:
				fprintf(stderr, "unknown acquire mode\n");
			}
		}
	}

	return ret;
}

static int HAL_open_nl_sock()
{
	int fd;
	int sndbuf = 32768;
	int rcvbuf = 32768;
	unsigned int addr_len;
	struct sockaddr_nl local;

	fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (fd < 0)
	{
		printf("Cannot open netlink socket");
		return -1;
	}

	setsockopt(fd, SOL_SOCKET, SO_SNDBUF, (void *)&sndbuf, sizeof(sndbuf));

	setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (void *)&rcvbuf, sizeof(rcvbuf));

	memset(&local, 0, sizeof(local));
	local.nl_family = AF_NETLINK;
	local.nl_groups = 0;

	if (bind(fd, (struct sockaddr *)&local, sizeof(local)) < 0)
	{
		printf("Cannot bind netlink socket");
		return -1;
	}

	addr_len = sizeof(local);
	if (getsockname(fd, (struct sockaddr *)&local, &addr_len) < 0)
	{
		printf("Cannot getsockname");
		return -1;
	}
	if (addr_len != sizeof(local))
	{
		printf("Wrong address length %u\n", addr_len);
		return -1;
	}
	if (local.nl_family != AF_NETLINK)
	{
		printf("Wrong address family %d\n", local.nl_family);
		return -1;
	}
	return fd;
}

static int HAL_get_link(const char *name, __u8 acquire, void *res)
{
	int err, fd;

	fd = HAL_open_nl_sock();
	if (fd < 0)
		return -1;

	err = HAL_do_get_nl_link(fd, acquire, name, res);
	close(fd);

	return err;
}

bool can_ifconfig_init(const char *device, int bitrate)
{
	FILE *fp = NULL;
	bool state = false;
	char buf[128] = {0};

	snprintf(buf, 127, "ip link set %s down", device);
	system(buf);
	snprintf(buf, 127, "ip link set %s type can bitrate %d dbitrate 500000 fd on", device, bitrate);
	system(buf);

	snprintf(buf, 127, "ip link set %s up", device);
	system(buf);

	// 检查网络是否存在
	snprintf(buf, 127, "ip link show %s", device);
	fp = popen(buf, "r");

	memset(buf, 0, 128);
	if (fgets(buf, 128, fp) != NULL)
	{
		state = true;
	}
	pclose(fp);

	return state;
}

bool can_band_init(const char *device, int *fd)
{
	struct ifreq m_ifr;
	struct sockaddr_can m_addr;

	// 创建 CAN 的套接字
	*fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (*fd < 0)
	{
		return false; // 创建套接字失败
	}
	int enable_fd = 1;
	if (setsockopt(*fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0)
	{
		close(*fd);	  // 关闭套接字
		return false; // 设置`CAN FD`模式失败
	}

	strcpy(m_ifr.ifr_name, device);
	if (ioctl(*fd, SIOCGIFINDEX, &m_ifr) < 0)
	{
		close(*fd);	  // 关闭套接字
		return false; // 获取接口索引失败
	}

	m_addr.can_family = AF_CAN;
	m_addr.can_ifindex = m_ifr.ifr_ifindex;

	if (bind(*fd, (struct sockaddr *)&m_addr, sizeof(m_addr)) < 0)
	{
		close(*fd);	  // 关闭套接字
		return false; // 绑定失败
	}
	int flags = fcntl(*fd, F_GETFL, 0);
	fcntl(*fd, F_SETFL, flags | O_NONBLOCK);

	return true; // 成功
}


bool HAL_canfd_write(int fd, struct canfd_frame *pFrame)
{
	int len;

	len = write(fd, pFrame, CANFD_MTU);

	if (len == sizeof(struct canfd_frame))
		return true;
	else
		return false;
}
bool HAL_can_write(int fd, struct can_frame *pFrame)
{
	int len;

	len = write(fd, pFrame, CAN_MTU);

	if (len == sizeof(struct can_frame))
		return true;
	else
		return false;
}

int HAL_canfd_read(int fd, struct canfd_frame *pFrame, unsigned int msTimeout)
{
	int len = read(fd, pFrame, sizeof(struct canfd_frame));
	// printf("pFrame->can_id : 0x%X\n",pFrame->can_id);
	if (len == sizeof(struct can_frame))
	{

		pFrame->flags = 0;
		return 1;
	}
	else if (len == sizeof(struct canfd_frame))
	{
		return 2;
	}
	else
	{
		if (len == -1 && errno == EAGAIN)
		{
			// 非阻塞下没有数据，epoll误触发或数据已读完
			printf("HAL_can_read timeout\n");
			return 0;  // 你可以换成 return true; 但说明没数据
		}
		else
		{
			perror("HAL_can_read failed");
			return 0;
		}
	}
	return 0;
}

int HAL_can_read(int fd, struct can_frame *pFrame, unsigned int msTimeout)
{
	int len = read(fd, pFrame, sizeof(struct can_frame));

	if (len == sizeof(struct can_frame))
	{
		return 1; // 成功读取一个标准 CAN 报文
	}
	else
	{
		if (len == -1 && errno == EAGAIN)
		{
			printf("HAL_can_read timeout (no data)\n");
			return 0; // 非阻塞无数据
		}
		else
		{
			perror("HAL_can_read failed");
			printf("HAL_can_read failed\n");
			return 0; // 读取失败
		}
	}
}

/*
 * 函数名称：can_close
 * 功能描述：CAN关闭
 * 输入参数：fd:SOCKET句柄
 * 输出参数：无
 */
void HAL_can_closeEx(int *fd)
{
	if (*fd >= 0)
	{
		close(*fd);
		*fd = -1;
	}
}

int HAL_can_get_state(const char *name, int *state)
{
	return HAL_get_link(name, GET_STATE, state);
}

