using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;

using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.IO.Ports;

public class RigControl : MonoBehaviour
{
	public GameObject humanoid;
	public Vector3 bodyRotation = new Vector3(0, 0, 0);
	RigBone leftUpperArm;
	RigBone leftLowerArm;
	RigBone rightUpperArm;
	RigBone rightLowerArm;
	// Start is called before the first frame update
	SerialPort sp = new SerialPort();
	void Start()
	{
		leftUpperArm = new RigBone(humanoid, HumanBodyBones.LeftUpperArm);
		leftLowerArm = new RigBone(humanoid, HumanBodyBones.LeftLowerArm);
		rightUpperArm = new RigBone(humanoid, HumanBodyBones.RightUpperArm);
		rightLowerArm = new RigBone(humanoid, HumanBodyBones.RightLowerArm);
		if (!sp.IsOpen)
		{
			sp.PortName = "COM8";
			sp.BaudRate = 2000000;
			sp.DataBits = 8;
			sp.Parity = Parity.None;
			sp.StopBits = StopBits.One;
			sp.Open();
			Debug.Log("Open Port");
		}
		else
		{
			Debug.Log("Opend Port");
		}
	}
	//In the following program, joints are bent using the RigBone.set(Quaternion) function. To rotate the entire Humanoid, set humanoid.transform.rotation to the value in the world coordinate system. The order of Rotation in Transform in Unity Editor is Y, X, Z in order, so apply here in that order as well.

	// Update is called once per frame
	float[] q0_init = new float[7];
	float[] q1_init = new float[7];
	float[] q2_init = new float[7];
	float[] q3_init = new float[7];

	float[] q0 = new float[7];
	float[] q1 = new float[7];
	float[] q2 = new float[7];
	float[] q3 = new float[7];
	bool start = false;
	void Update()
	{
		string recv_data_0 = "";
		int bytesize = sp.BytesToRead;
		byte[] tt = new byte[bytesize];
		sp.Read(tt, 0, bytesize);
		for (int i = 0; i < bytesize; i++)
		{
			recv_data_0 += Convert.ToChar(tt[i]);
		}
		//        if (bytesize == 19 && (recv_data_0[0] == 's') && Convert.ToChar(recv_data_0[1]) != 0 && (recv_data_0[5] == ',') && (recv_data_0[9] == ',') && (recv_data_0[13] == ','))
		if (bytesize == 19 && (recv_data_0[0] == 's') && (recv_data_0[5] == ',') && (recv_data_0[9] == ',') && (recv_data_0[13] == ','))
		{//s2300,100,100,100
			int value = Convert.ToChar(recv_data_0[1]) - 48;
			if (!start)
			{
				Debug.Log(recv_data_0);
				Debug.Log(value + "  " + string.Format("{0:F3}", q0[value]) + "  " + string.Format("{0:F3}", q1[value]) + "  " + string.Format("{0:F3}", q2[value]) + "  " + string.Format("{0:F3}", q3[value]));
			}
			q0[value] = (((Convert.ToChar(recv_data_0[2]) - 48) * 100 + (Convert.ToChar(recv_data_0[3]) - 48) * 10 + (Convert.ToChar(recv_data_0[4]) - 48)) / 100f) - 1f;
			q1[value] = (((Convert.ToChar(recv_data_0[6]) - 48) * 100 + (Convert.ToChar(recv_data_0[7]) - 48) * 10 + (Convert.ToChar(recv_data_0[8]) - 48)) / 100f) - 1f;
			q2[value] = (((Convert.ToChar(recv_data_0[10]) - 48) * 100 + (Convert.ToChar(recv_data_0[11]) - 48) * 10 + (Convert.ToChar(recv_data_0[12]) - 48)) / 100f) - 1f;
			q3[value] = (((Convert.ToChar(recv_data_0[14]) - 48) * 100 + (Convert.ToChar(recv_data_0[15]) - 48) * 10 + (Convert.ToChar(recv_data_0[16]) - 48)) / 100f) - 1f;

			if (Input.GetKey(KeyCode.Space))
			{
				start = true;
				Debug.Log("QUATERNION ANGLE INIT\n");
				for (int i = 1; i < 7; i++)
				{
					q0_init[i] = q0[i];
					q1_init[i] = q1[i];
					q2_init[i] = q2[i];
					q3_init[i] = q3[i];
					Debug.Log(q0_init[i] + "  " + q1_init[i] + "  " + q2_init[i] + "  " + q3_init[i]);
				}
			}
			if (start)
			{
				Debug.Log("leftUpperArm	   :" + "  " + string.Format("{0:F3}", q0[1]) + "  " + string.Format("{0:F3}", q1[1]) + "  " + string.Format("{0:F3}", q2[1]) + "  " + string.Format("{0:F3}", q3[1]));
				Debug.Log("leftLowerArm	   :" + "  " + string.Format("{0:F3}", q0[2]) + "  " + string.Format("{0:F3}", q1[2]) + "  " + string.Format("{0:F3}", q2[2]) + "  " + string.Format("{0:F3}", q3[2]));
				Debug.Log("rightUpperArm   :" + "  " + string.Format("{0:F3}", q0[3]) + "  " + string.Format("{0:F3}", q1[3]) + "  " + string.Format("{0:F3}", q2[3]) + "  " + string.Format("{0:F3}", q3[3]));
				Debug.Log("rightLowerArm   :" + "  " + string.Format("{0:F3}", q0[4]) + "  " + string.Format("{0:F3}", q1[4]) + "  " + string.Format("{0:F3}", q2[4]) + "  " + string.Format("{0:F3}", q3[4]));
				Quaternion relative_1 = Quaternion.Inverse(new Quaternion(q1_init[1], (-1) * q2_init[1], q3_init[1], (-1) * q0_init[1])) * new Quaternion(q1[1], (-1) * q2[1], q3[1], (-1) * q0[1]);
				Quaternion relative_2 = Quaternion.Inverse(new Quaternion(q1_init[2], (-1) * q2_init[2], q3_init[2], (-1) * q0_init[2])) * new Quaternion(q1[2], (-1) * q2[2], q3[2], (-1) * q0[2]);
				Quaternion relative_3 = Quaternion.Inverse(new Quaternion(q1_init[3], (-1) * q2_init[3], q3_init[3], (-1) * q0_init[3])) * new Quaternion(q1[3], (-1) * q2[3], q3[3], (-1) * q0[3]);
				Quaternion relative_4 = Quaternion.Inverse(new Quaternion(q1_init[4], (-1) * q2_init[4], q3_init[4], (-1) * q0_init[4])) * new Quaternion(q1[4], (-1) * q2[4], q3[4], (-1) * q0[4]);
				leftUpperArm.set(relative_2);
				leftLowerArm.set(relative_1);
				rightUpperArm.set(relative_3);
				rightLowerArm.set(relative_4);
				humanoid.transform.rotation
				  = Quaternion.AngleAxis(bodyRotation.z, new Vector3(0, 0, 1))
				  * Quaternion.AngleAxis(bodyRotation.x, new Vector3(1, 0, 0))
				  * Quaternion.AngleAxis(bodyRotation.y, new Vector3(0, 1, 0));
			}
		}
	}
}
